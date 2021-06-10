/**
 * Communication is done via frames of fixed size (SPI_HEADER_SIZE + SPI_BLOCK_SIZE)
 * Header contains some meta information while block is actual user data.
 * Header format:
 * offset   type    description
 * 0x00     u32     CRC32 of all bytes from offset 0x04 to the end of block
 * 0x04     u16     Header size (offset to block/data)
 * 0x06     u16     Block/data size
 * 0x08     u32     Flags for this packet
 * 0x0C     u8      Frame id. Used for frame loss detection
 * 0x0D     u8      Number of elements in "last received frames" array (0..10)
 * 0x0E     u8[8]   Ids of frames that were received last
 * 0x20     u8      Number of elements in "lost frames" array (0..8)
 * 0x21     u8[8]   Ids of lost frames
 */
#include "spi_protocol.h"

#include <string.h>
#include <crc.h>

#define OFFSET_HEADER_SIZE  0x04
#define OFFSET_DATA_SIZE    0x06
#define OFFSET_FLAGS        0x08
#define OFFSET_FRAME_ID     0x0C
#define OFFSET_RECEIVED_LEN 0x0D
#define OFFSET_RECEIVED_ARR 0x0E
#define OFFSET_LOST_LEN     0x20
#define OFFSET_LOST_ARR     0x21


#define SPI_HEADER_SIZE 64
#define SPI_PADDING_SIZE 4

#define SPI_TX_BUFFER_SIZE (SPI_HEADER_SIZE + SPI_BLOCK_SIZE + SPI_PADDING_SIZE)
#define SPI_RX_BUFFER_SIZE (SPI_HEADER_SIZE + SPI_BLOCK_SIZE)

/**
 * Indicates that block doesn't carry any data and only carries header
 */
#define SPI_FLAG_HEADER_ONLY 0x01u

typedef struct {
    uint16_t headerSize;
    uint32_t flags;
    uint8_t id;

    uint8_t received[SPI_RX_HISTORY];
    uint8_t receivedLen;

    uint8_t lost[SPI_RX_HISTORY];
    uint8_t lostLen;

    uint8_t *data;
    uint16_t dataSize;
} SpiFrame;


#if SPI_CHANNELS < 1 || SPI_CHANNELS > 2
#error Unsupported number of channels
#endif
#if SPI_CHANNELS > 0
static SpiProtocol channel0;
static uint8_t channel0tx[SPI_TX_BUFFER_SIZE] __attribute__ ((aligned (SPI_MEM_ALIGN)));
static uint8_t channel0rx[SPI_RX_BUFFER_SIZE] __attribute__ ((aligned (SPI_MEM_ALIGN)));
static uint8_t channel0mem[SPI_BLOCK_SIZE * (SPI_TX_BLOCKS + SPI_RX_BLOCKS)];
#endif
#if SPI_CHANNELS > 1
static SpiProtocol channel1;
static uint8_t channel1tx[SPI_TX_BUFFER_SIZE] __attribute__ ((aligned (SPI_MEM_ALIGN)));
static uint8_t channel1rx[SPI_RX_BUFFER_SIZE] __attribute__ ((aligned (SPI_MEM_ALIGN)));
static uint8_t channel1mem[SPI_BLOCK_SIZE * (SPI_TX_BLOCKS + SPI_RX_BLOCKS)];
#endif

/**
 * Internal function, called only when channel is locked
 * @param spi
 */
static void channelTick(SpiProtocol *spi);

static void initMemory(SpiProtocol *spi, uint8_t *tx, uint8_t *rx, uint8_t *mem);

static void processTxQueue(SpiProtocol *spi);

static void processRxBuffer(SpiProtocol *spi);

//static void writeHeader(void *buffer, SpiFrameHeader header);
static void writeFrame(uint8_t *buffer, SpiFrame frame);

static bool readFrame(uint8_t *buffer, SpiFrame *frame);

static bool getTxBlock(SpiProtocol *spi, SpiProtocolBlock **block,
                       SpiProtocolBlockState state);

SpiProtocol *spiProtocolInitialize(SpiProtocolInit *init, uint8_t channel) {
    init->setTxBusy(false);
    init->setRxBusy(false);
    if (channel == 0) {
        initMemory(&channel0, channel0tx, channel0rx, channel0mem);
        memcpy(&channel0.init, init, sizeof(SpiProtocolInit));
        return &channel0;
    }
#if SPI_CHANNELS > 1
    else if (channel == 1) {
        initMemory(&channel1, channel1tx, channel1rx, channel1mem);
        memcpy(&channel1.init, init, sizeof(SpiProtocolInit));
        return &channel1;
    }
#endif

    return NULL;
}

void spiProtocolTick(SpiProtocol *spi) {
    if (!spi->init.lock())
        return;
    channelTick(spi);
    spi->init.unlock();
}

void spiProtocolWrite(SpiProtocol *spi, const void *data, uint32_t size) {
    const uint8_t *d = data;
    uint32_t sizeLeft = size;
    SpiProtocolBlock *block;

    while (!spi->init.lock()) {
        spi->init.delay(1);
    }
    while (sizeLeft > 0) {
        channelTick(spi);
        if (!getTxBlock(spi, &block, BLOCK_STATE_FREE)) {
            spi->init.delay(1);
            continue;
        }

        uint32_t dataSize = sizeLeft < SPI_BLOCK_SIZE ? sizeLeft : SPI_BLOCK_SIZE;
        memcpy(block->buffer, d, dataSize);

        block->size = dataSize;
        block->state = BLOCK_STATE_TX_READY;

        sizeLeft -= dataSize;
        d += dataSize;
    }
    spi->init.unlock();
    // wait until all blocks are received by the other end
    bool allReceived = false;
    while (!allReceived) {
        allReceived = true;
        for (int i = 0; i < SPI_TX_BLOCKS; ++i) {
            if (spi->txBlocks[i].state != BLOCK_STATE_FREE) {
                allReceived = false;
                break;
            }
        }
        spiProtocolTick(spi);
        spi->init.delay(1);
    }
}

static void channelTick(SpiProtocol *spi) {
    if (spi->rxPending) {
        SCB_InvalidateDCache_by_Addr((uint32_t *) spi->rxBuffer, SPI_RX_BUFFER_SIZE);
        processRxBuffer(spi);

        memset(spi->rxBuffer, 0, SPI_RX_BUFFER_SIZE);
        // wrote data (with memset) to memory which is used by DMA, need to clean DCache
        SCB_CleanDCache_by_Addr((uint32_t *) spi->rxBuffer, SPI_RX_BUFFER_SIZE);
        spi->rxPending = false;
    }

    // check if we can begin receiving
    if (spi->init.rx->State == HAL_SPI_STATE_READY && !spi->init.isRxBusy()) {
        // we are ready to receive and other side is not sending

        HAL_SPI_Receive_DMA(spi->init.rx, spi->rxBuffer, SPI_RX_BUFFER_SIZE);
        spi->init.setRxBusy(true);
    }

    // process tx queue when we can transmit
    if (spi->init.tx->State == HAL_SPI_STATE_READY && spi->init.isTxBusy()) {
        // we are ready to transmit and other side is listening
        processTxQueue(spi);
    }
}

static void initMemory(SpiProtocol *spi, uint8_t *tx, uint8_t *rx, uint8_t *mem) {
    memset(spi, 0, sizeof(SpiProtocol));
    spi->txBuffer = tx;
    spi->rxBuffer = rx;

    for (int i = 0; i < SPI_TX_BLOCKS; ++i) {
        spi->txBlocks[i].buffer = mem;
        spi->txBlocks[i].size = SPI_BLOCK_SIZE;
        spi->txBlocks[i].state = BLOCK_STATE_FREE;
        mem += SPI_BLOCK_SIZE;
    }
    for (int i = 0; i < SPI_RX_BLOCKS; ++i) {
        spi->rxBlocks[i].buffer = mem;
        spi->rxBlocks[i].size = SPI_BLOCK_SIZE;
        spi->rxBlocks[i].state = BLOCK_STATE_FREE;
        spi->rxBlocks[i].frameId = i;
        mem += SPI_BLOCK_SIZE;
    }
}

static void processTxQueue(SpiProtocol *spi) {
    SpiProtocolBlock *block;
    bool isHeaderOnly = !getTxBlock(spi, &block, BLOCK_STATE_TX_READY) &&
                        !getTxBlock(spi, &block, BLOCK_STATE_RESEND) &&
                        !getTxBlock(spi, &block, BLOCK_STATE_SENT);
    if (isHeaderOnly && !spi->txRequired) {
        return;
    }

    if (!isHeaderOnly && block->state == BLOCK_STATE_TX_READY) {
        // check if we have block with the same id in not yet confirmed blocks
        block->frameId = spi->nextFrameId;
        for (int i = 0; i < SPI_TX_BLOCKS; ++i) {
            if (spi->txBlocks[i].state == BLOCK_STATE_SENT && spi->txBlocks[i].frameId == block->frameId) {
                isHeaderOnly = true;
                break;
            }
        }
        if (!isHeaderOnly) {
            ++spi->nextFrameId;
        }
    }
    if (!isHeaderOnly) {
        block->state = BLOCK_STATE_SENT;
    }

    SpiFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.headerSize = SPI_HEADER_SIZE;
    frame.receivedLen = spi->recentFramesLen;
    memcpy(frame.received, spi->recentFrames, SPI_RX_HISTORY);
    frame.lostLen = spi->lostFramesLen;
    memcpy(frame.lost, spi->lostFrames, SPI_RX_HISTORY);

    if (isHeaderOnly) {
        frame.flags |= SPI_FLAG_HEADER_ONLY;
    } else {
        frame.id = block->frameId;
        frame.data = block->buffer;
        frame.dataSize = block->size;
    }

    writeFrame(spi->txBuffer, frame);

    uint32_t crc = crc32pre();
    crc = crc32upd(&spi->txBuffer[4], SPI_HEADER_SIZE - 4, crc);
    if (!isHeaderOnly) {
        crc = crc32upd(block->buffer, block->size, crc);
    }
    *(uint32_t *) spi->txBuffer = crc32post(crc);

    SCB_CleanDCache_by_Addr((uint32_t *) spi->txBuffer, SPI_TX_BUFFER_SIZE);
    spi->init.setTxBusy(true);
    HAL_SPI_Transmit_DMA(spi->init.tx, spi->txBuffer, SPI_TX_BUFFER_SIZE);
    spi->txRequired = false;
}

static void processRxBuffer(SpiProtocol *spi) {
    SpiFrame frame;

    if (!readFrame(spi->rxBuffer, &frame)) {
        ++spi->receivedCorruptedFrames;
        return;
    }

    bool isHeaderOnly = frame.flags & SPI_FLAG_HEADER_ONLY;
    // we should always answer with something if we received data
    spi->txRequired = !isHeaderOnly;

    if (!isHeaderOnly) {
        // rxBlocks is prefilled with expected ids, so if we find frame id there
        // we just need to copy it
        for (int i = 0; i < SPI_RX_BLOCKS; ++i) {
            SpiProtocolBlock *block = &spi->rxBlocks[i];
            if (block->frameId == frame.id) {
                block->size = frame.dataSize;
                memcpy(block->buffer, frame.data, block->size);
                block->state = BLOCK_STATE_RECEIVED;

                if (spi->recentFramesLen == SPI_RX_HISTORY) {
                    memmove(spi->recentFrames, &spi->recentFrames[1], (SPI_RX_HISTORY - 1));
                    spi->recentFrames[SPI_RX_HISTORY - 1] = frame.id;
                } else {
                    spi->recentFrames[spi->recentFramesLen] = frame.id;
                    ++spi->recentFramesLen;
                }
                break;
            }
        }
    }

    // send all data in correct order to user
    int blocksRemoved = 0;
    for (int i = 0; i < SPI_RX_BLOCKS; ++i) {
        SpiProtocolBlock *block = &spi->rxBlocks[i];
        if (block->state != BLOCK_STATE_RECEIVED) {
            break;
        } else {
            spi->init.onData(block->buffer, block->size);
            ++spi->receivedFrames;
            ++blocksRemoved;
        }
    }

    if (blocksRemoved > 0) {
        for (int i = 0; i < SPI_RX_BLOCKS - blocksRemoved; ++i) {
            uint8_t *oldData = spi->rxBlocks[i].buffer;
            memcpy(&spi->rxBlocks[i], &spi->rxBlocks[i + blocksRemoved], sizeof(SpiProtocolBlock));
            spi->rxBlocks[i + blocksRemoved].state = BLOCK_STATE_FREE;
            spi->rxBlocks[i + blocksRemoved].buffer = oldData;
        }
        uint8_t frameId = spi->rxBlocks[0].frameId;
        if (blocksRemoved == SPI_RX_BLOCKS) {
            frameId = spi->rxBlocks[SPI_RX_BLOCKS - 1].frameId + 1;
        }
        for (int i = 0; i < SPI_RX_BLOCKS; ++i) {
            spi->rxBlocks[i].frameId = frameId;
            if (i >= SPI_RX_BLOCKS - blocksRemoved) {
                spi->rxBlocks[i].state = BLOCK_STATE_FREE;
            }
            ++frameId;
        }
    }

    // update lost blocks information
    spi->lostFramesLen = 0;
    bool foundReady = false;
    for (int i = SPI_RX_BLOCKS - 1; i >= 0; --i) {
        SpiProtocolBlock *block = &spi->rxBlocks[i];
        if (block->state != BLOCK_STATE_FREE) {
            foundReady = true;
        } else if (foundReady) {
            // this frame id is missing, add to list of missing blocks
            spi->lostFrames[spi->lostFramesLen] = block->frameId;
            ++spi->lostFramesLen;
        }
    }
    if (!foundReady && isHeaderOnly) {
        // by default set next block as "lost" if current rx
        // block was empty
        spi->lostFrames[0] = spi->rxBlocks[0].frameId;
        spi->lostFramesLen = 1;
    }


    // process information about received and lost frames by other side
    for (int i = 0; i < frame.receivedLen; ++i) {
        for (int j = 0; j < SPI_TX_BLOCKS; ++j) {
            if (spi->txBlocks[j].state == BLOCK_STATE_SENT &&
                spi->txBlocks[j].frameId == frame.received[i]) {
                spi->txBlocks[j].state = BLOCK_STATE_FREE;
            }
        }
    }

    for (int i = 0; i < frame.lostLen; ++i) {
        for (int j = 0; j < SPI_TX_BLOCKS; ++j) {
            if (spi->txBlocks[j].state == BLOCK_STATE_SENT &&
                spi->txBlocks[j].frameId == frame.lost[i]) {
                spi->txBlocks[j].state = BLOCK_STATE_RESEND;
            }
        }
    }
    // compact txBlocks so there are no free blocks between sent/ready
    int j = 0;
    for (int i = 0; i < SPI_TX_BLOCKS; ++i) {
        if (i != j && spi->txBlocks[i].state != BLOCK_STATE_FREE) {
            uint8_t *oldData = spi->txBlocks[j].buffer;
            memcpy(&spi->txBlocks[j], &spi->txBlocks[i], sizeof(SpiProtocolBlock));
            spi->txBlocks[i].state = BLOCK_STATE_FREE;
            spi->txBlocks[i].buffer = oldData;
        }
        if (spi->txBlocks[j].state != BLOCK_STATE_FREE) {
            ++j;
        }
    }
}

static void writeFrame(uint8_t *buffer, SpiFrame frame) {
    memset(buffer, 0, frame.headerSize);

    *(uint16_t *) &buffer[OFFSET_HEADER_SIZE] = frame.headerSize;

    buffer[OFFSET_RECEIVED_LEN] = frame.receivedLen;
    memcpy(&buffer[OFFSET_RECEIVED_ARR], frame.received, frame.receivedLen);
    buffer[OFFSET_LOST_LEN] = frame.lostLen;
    memcpy(&buffer[OFFSET_LOST_ARR], frame.lost, frame.lostLen);

    *(uint32_t *) &buffer[OFFSET_FLAGS] = frame.flags;

    buffer[OFFSET_FRAME_ID] = frame.id;
    *(uint16_t *) &buffer[OFFSET_DATA_SIZE] = frame.dataSize;

    if (frame.dataSize > 0) {
        memcpy(&buffer[SPI_HEADER_SIZE], frame.data, frame.dataSize);
    }
}

static bool readFrame(uint8_t *buffer, SpiFrame *frame) {
    frame->dataSize = *(uint16_t *) &buffer[OFFSET_DATA_SIZE];
    frame->headerSize = *(uint16_t *) &buffer[OFFSET_HEADER_SIZE];

    if (frame->headerSize<SPI_HEADER_SIZE || frame->headerSize + frame->dataSize>SPI_RX_BUFFER_SIZE) {
        // sizes can't be valid
        return false;
    }
    // sizes look reasonable, check CRC
    uint32_t crc = *(uint32_t *) buffer;
    uint32_t actualCrc = crc32(&buffer[4], frame->headerSize + frame->dataSize - 4);

    if (crc != actualCrc) {
        // corrupted frame, skip it
        return false;
    }

    frame->id = buffer[OFFSET_FRAME_ID];
    frame->flags = *(uint32_t *) &buffer[OFFSET_FLAGS];

    frame->data = &buffer[frame->headerSize];
    memcpy(frame->lost, &buffer[OFFSET_LOST_ARR], SPI_RX_HISTORY);
    frame->lostLen = buffer[OFFSET_LOST_LEN];

    memcpy(frame->received, &buffer[OFFSET_RECEIVED_ARR], SPI_RX_HISTORY);
    frame->receivedLen = buffer[OFFSET_RECEIVED_LEN];

    return true;
}

static bool getTxBlock(SpiProtocol *spi, SpiProtocolBlock **block, SpiProtocolBlockState state) {
    *block = NULL;
    for (int i = 0; i < SPI_TX_BLOCKS; ++i) {
        if (spi->txBlocks[i].state == state) {
            *block = &spi->txBlocks[i];
            return true;
        }
    }
    return false;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    SpiProtocol *spi;
#if SPI_CHANNELS > 0
    if (channel0.init.rx == hspi) {
        spi = &channel0;
    }
#endif
#if SPI_CHANNELS > 1
    else if (channel1.init.rx == hspi) {
        spi = &channel1;
    }
#endif
    else {
        return;
    }

    spi->init.setRxBusy(false);
    spi->rxPending = true;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    SpiProtocol *spi;
#if SPI_CHANNELS > 0
    if (channel0.init.tx == hspi) {
        spi = &channel0;
    }
#endif
#if SPI_CHANNELS > 1
    else if (channel1.init.tx == hspi) {
        spi = &channel1;
    }
#endif
    else {
        return;
    }

    spi->init.setTxBusy(false);
    ++spi->sentFrames;
}
