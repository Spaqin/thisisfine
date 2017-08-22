#ifndef FINEPROTO_H
#define FINEPROTO_H
#define QUEUE_SIZE 8

typedef enum Sensor {
  Temperature = 0x0,
  Humidity = 0x1,
  PM25 = 0x2,
  PM10 = 0x3,
  CO = 0x4
};

typedef struct FineMessage {
    char header;
    char command;
    char data[2];
    char checksum;
};

typedef struct FineProtocol {
    // Receiving-related:
    FineMessage last_rcv;  // for DMA - will copy from here to buffer after it's received.
    FineMessage rcv_queue[QUEUE_SIZE];  // circular buffer/queue.
    uint8_t rdi;  // read iterator - where the uc is at now with parsing
    uint8_t rci;  // receive iterator - the last message rcvd and copied
    // Sending-related:
    FineMessage to_send; 
    uint16_t (*get_data_for[16])(void);  // callbacks for sensor data
    // Misc:
    uint16_t continuous_timer; // counter for timer [in seconds]
    uint8_t pri;  // protocol iterator for continous mode
};

FineProtocol _fineproto;

void fineproto_add_sensor(uint16_t (*get_data_for)(void));  // add sensor callback
void fineproto_init();  // clean up the struct
void parse_all_messages();  // goes through all messages in the queue
void parse_message(FineMessage);  // sends an appropriate message or sets internal values accordingly.
void _continous_setup();  // sets up the timer for continous mode
void _continous_advance();  // sends and advances iterator for cont mode; called back when the timer calls
void _got_message(); // callback for DMA finished (received a message)
FineMessage _create_data_message(Sensor);  // tailors a message for given proto number
inline uint8_t _calculate_checksum(FineMessage);

#endif
