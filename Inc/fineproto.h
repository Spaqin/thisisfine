#ifndef FINEPROTO_H
#define FINEPROTO_H
#define QUEUE_SIZE 8
#define FINEPROTO_HEADER 0xF1
#define FINEPROTO_QUERY 0xF2

#define QUERY_CAPABILITIES 0x01
#define QUERY_SENSOR  0x02  //0xX2 - X is sensor ID
#define QUERY_FWVER 0x04
#define QUERY_CONTINOUS 0x05
#define QUERY_HANDSHAKE 0xF1

#ifndef BUILDDAY
#define BUILDDAY 1
#endif

#ifndef BUILDMONTH
#define BUILDMONTH 9
#endif

typedef enum {
  Temperature = 0x0,
  Humidity = 0x1,
  PM25 = 0x2,
  PM10 = 0x3,
  CO = 0x4
} Sensor;

typedef struct {
    char header;
    char command;
    char data[2];
    char checksum;
} FineMessage;

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
    uint16_t continuous_timer; // counter for timer [in seconds], btw i hate how it's not just "continous", but someone put an extra u in there for no reason at all goddamn
    uint8_t pri;  // protocol iterator for continous mode
} FineProtocol;

FineProtocol _fineproto;

void fineproto_add_sensor(uint16_t (*get_data_for)(void));  // add sensor callback
void fineproto_init();  // clean up the struct
void fp_parse_all_messages();  // goes through all messages in the queue
void fp_parse_message(FineMessage);  // sends an appropriate message or sets internal values accordingly.
void _fp_continuous_setup();  // sets up the timer for continous mode
void _fp_continuous_stop();  // disables timer, cleans up
void _fp_continuous_advance();  // sends and advances iterator for cont mode; called back when the timer calls
void _fp_got_message(); // callback for DMA finished (received a message)
FineMessage _fp_create_data_message(Sensor);  // tailors a message for given proto number
inline uint8_t _fp_calculate_checksum(FineMessage);

#endif
