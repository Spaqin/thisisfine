#define QUEUE_SIZE 8


typedef struct FineMessage {
    char header;
    char command;
    char data[2];
    char checksum;
};

typedef struct FineProtocol {
    // Receiving-related:
    FineMessage rcv_queue[QUEUE_SIZE];  //circular buffer/queue.
    uint8_t rdi;  //read iterator - where the uc is at now with parsing
    uint8_t rci;  //receive iterator - the last message rcvd
    // Sending-related:
    FineMessage to_send; 
    uint8_t pri;  //protocol iterator for continous mode
    uint16_t (*get_data_for[16])(void);  //callbacks for sensor data
    // Misc:
    uint16_t continuous_timer; //counter for timer [in seconds]
};

FineProtocol _fineproto;

void parse_all_messages();  //goes through all messages in the queue
void parse_message(FineMessage);  //sends an appropriate message or sets internal values accordingly.
void _continous_setup();  //sets up the timer for continous mode
void _continous_advance();  //sends and advances iterator for cont mode; called back when the timer calls
void _got_message(); //callback for DMA finished (received a message)


