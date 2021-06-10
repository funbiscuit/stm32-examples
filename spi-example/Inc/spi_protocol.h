#ifndef H7_SPI_EXAMPLE_SPI_PROTOCOL_H
#define H7_SPI_EXAMPLE_SPI_PROTOCOL_H

#include <stdbool.h>
#include <inttypes.h>

#include <stm32h7xx_hal.h>

#define SPI_TX_BLOCKS 4
#define SPI_RX_BLOCKS 4

#define SPI_RX_HISTORY 10

/**
 *
 */
#define SPI_BLOCK_SIZE (8*1024)
//#define SPI_BLOCK_SIZE (64)

/**
 * Number of channels that need to be created
 */
#define SPI_CHANNELS 2

/**
 * Indicates that DCache is used and it should be
 * cleaned/invalidated after DMA finishes
 */
#define SPI_CM7_CACHE_USED

/**
 * Boundary (in bytes) to which memory blocks must be aligned
 * Required for DCache invalidate
 */
#define SPI_MEM_ALIGN 32

typedef struct {
    /**
     * Handle to SPI that is configured as transmit only master (8bit data)
     * with DMA in normal mode (configured with byte size).
     * Hardware SS is disabled
     */
    SPI_HandleTypeDef *tx;

    /**
     * Handle to SPI that is configured as receive only slave (8bit data)
     * with DMA in normal mode (configured with byte size).
     * Hardware SS is disabled
     */
    SPI_HandleTypeDef *rx;

    /**
     * Callback for received data
     * @param data
     * @param size
     */
    void (*onData)(const uint8_t *data, uint32_t size);

    /**
     * Function to delay execution when protocol has nothing to do but
     * needs to block execution
     * @param ms - milliseconds to delay
     */
    void (*delay)(uint32_t ms);

    /**
     * Function that sets RX busy line state.
     * Line must be implemented in such way that isRxBusy
     * function will read logical AND of two states.
     * For example, it can be configure as open drain with pull up
     * And low state will mean its busy
     */
    void (*setRxBusy)(bool);

    /**
     * Function that sets TX busy line state.
     * Line must be implemented in such way that isTxBusy
     * function will read logical AND of two states.
     * For example, it can be configure as open drain with pull up
     * And low state will mean its busy
     */
    void (*setTxBusy)(bool);

    /**
     * Returns state of RX busy line (logical AND of two sides)
     */
    bool (*isRxBusy)(void);

    /**
     * Returns state of TX busy line (logical AND of two sides)
     */
    bool (*isTxBusy)(void);
} SpiProtocolInit;

typedef enum {
    BLOCK_STATE_FREE,
    BLOCK_STATE_SENT,
    BLOCK_STATE_RESEND,
    BLOCK_STATE_TX_READY,
    BLOCK_STATE_RECEIVED,
} SpiProtocolBlockState;

typedef struct {
    uint8_t *buffer;
    uint16_t size;

    /**
     * Id of frame that carried this block
     * Valid when state becomes BLOCK_STATE_SENT
     */
    uint8_t frameId;

    SpiProtocolBlockState state;
} SpiProtocolBlock;


typedef struct {
    /**
     * Only fields in init struct can be set by user
     */
    SpiProtocolInit init;

    SpiProtocolBlock txBlocks[SPI_TX_BLOCKS];

    SpiProtocolBlock rxBlocks[SPI_RX_BLOCKS];

    uint8_t *txBuffer;
    uint8_t *rxBuffer;

    /**
     * When true, there is data in rxBuffer that needs to be processed
     */
    bool rxPending;

    /**
     * When true, we need to transfer at least header only frame
     * so other side knows what frames we received and what is lost
     */
    bool txRequired;

    /**
     * Ids of frames that were lost/corrupted
     */
    uint8_t lostFrames[SPI_RX_HISTORY];
    uint8_t lostFramesLen;

    /**
     * Ids of frames that were received recently
     */
    uint8_t recentFrames[SPI_RX_HISTORY];
    uint8_t recentFramesLen;

    uint8_t nextFrameId;

    uint64_t sentFrames;
    uint64_t receivedFrames;
    uint64_t receivedCorruptedFrames;
} SpiProtocol;

/**
 * This function must be called before any communication
 * so protocol can initialize internal buffers from init configuration
 * @param spi
 * @param channel - what channel to use (determines memory blocks used for
 * protocol)
 * @return non null if successful
 */
SpiProtocol *spiProtocolInitialize(SpiProtocolInit *init, uint8_t channel);

/**
 * Should be called so protocol can do its logic for
 * tx/rx control
 * @param spi
 */
void spiProtocolTick(SpiProtocol *spi);

/**
 * Write data
 * This operation blocks until all data is successfully written
 * @param spi
 * @param data
 * @param size
 */
void spiProtocolWrite(SpiProtocol *spi, const void *data, uint32_t size);

#endif //H7_SPI_EXAMPLE_SPI_PROTOCOL_H
