// message.h
#ifndef MESSAGE_H // include guard
#define MESSAGE_H

// Define the Message type
struct Message
{
    int source_id; // source node id
    int destination_id; // destination node id
    int data; // some data

    // Constructor with no parameters
    Message()
    {
        source_id = 0;
        destination_id = 0;
        data = 0;
    }

    // Constructor with three parameters
    Message(int s, int d, int x)
    {
        source_id = s;
        destination_id = d;
        data = x;
    }
};

// Define the CtrlPkt type
struct CtrlPkt
{
	int src_id; // source node id
    int dest_id; // destination node id
	Message msg; // A Message object as a member of CtrlPkt
    bool is_rts; // true if RTS, false if CTS
    bool is_fd; // true if full-duplex, false if half-duplex
    int ao; // number of allocated slots
    std::vector<int> reserved_slots; // vector of reserved slots
    std::vector<int> control_slots; // vector of control slots
    CtrlPkt() : control_slots(0) {} // Default constructor with no parameters

    CtrlPkt(Message &m) : msg(m), control_slots(0) {} // Constructor that takes a Message object as a parameter

    CtrlPkt(int num_slots) : control_slots(num_slots, 0) {} // Initialize the vector in the constructor using num_slots parameter

    CtrlPkt(Message &m, int num_slots) : msg(m), control_slots(num_slots, 0) {} // Initialize the vector and copy the fields from Message object

    CtrlPkt(Message &m, bool rts, bool fd, int ao, int num_slots) : msg(m), is_rts(rts), is_fd(fd), ao(ao), control_slots(num_slots, 0) {} // Another constructor that takes more parameters

    
};

#endif // MESSAGE_H
