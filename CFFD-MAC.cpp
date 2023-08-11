// C++ code for CFFD-MAC protocol
#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <chrono>
#include <set>
#include <queue>
#include <cmath>
#include "message.h"

using namespace std;
#define SENDING 1
#define RECEIVING 2

// Simulation parameters
const int num_nodes = 5;
const int net_area = 400;
const int tx_range = 200;
const int pkt_len = 1000;
const double frame_len = 0.002;
const int num_slots = 15;
const double cw_min = 0.000015;
const double data_rate = 11e6;
const double slot_time = 0.000015;
const double difs_time = 0.0001;
const double sifs_time = 0.00005;
const int retrans_limit = 3;


// Declare control_slots as a global variable
vector<int> control_slots;

struct cfd {
  int src_id;
  int dest_id;
};


struct Edge {
  int src_id;
  int dest_id;
};

// Data packet structure
struct DataPkt
{
	int src_id;	// source node id
	int dest_id;	// destination node id
	vector<char> payload;	// payload of the packet (random bytes)
    std::vector<std::set < int>> data_slots; // Declare the vector without size

    DataPkt() : data_slots(num_nodes) {} // Initialize the vector in the constructor using num_nodes variable
};

struct Node
{
    int id;
    int x;
    int y;
    int status;
    int slot;
    int trans_range;
    std::queue<Message> queue;
    bool has_pkt;
    int dest_id; 
    double wait_time;
    int backoff_count;
    int retrans_count;
    bool is_sender;
    bool is_neighbor;
    vector<int> data_slots;
};


struct FbPkt
{
    int src_id;      // source node id
    int dest_id;     // destination node id
    bool is_ack;     // whether packet is an ACK or not
};

// Global variables
vector<Node> nodes;
vector<Message> ctrl_pkts;
vector<Message> data_pkts;

// Generate packets with probability p in each control slot
void generate_pkts(int num_nodes, double p)
{
    for (int i = 0; i < num_nodes; i++)
    {
        // Generate a random number between 0 and 1
        double r = (double)rand() / RAND_MAX;
        // If the random number is less than p, generate a packet
        if (r < p)
        {
            // Choose a random destination node
            int dest = rand() % num_nodes;
            // Avoid self-loop
            while (dest == i)
            {
                dest = rand() % num_nodes;
            }
            // Create a message object with source, destination and data
            Message msg(i, dest, 1);
            // Push the message to the queue of node i
            nodes[i].queue.push(msg);
        }
    }
}

// Function to generate a random number in a given range
double rand_range(double min, double max)
{
    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> dis(min, max);
    return dis(gen);
}

// Function to generate a random byte (char)
char rand_byte()
{
    return static_cast<char>(rand_range(0, 256));
}

// Function to generate a random packet payload of a given length
vector<char> rand_payload(int len)
{
    vector<char> payload;
    for (int i = 0; i < len; i++)
    {
        payload.push_back(rand_byte());
    }
    return payload;
}

// Function to check if two nodes are within transmission range of each other
bool is_in_range(Node &n1, Node &n2)
{
    double dist = sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
    return dist <= n1.trans_range && dist <= n2.trans_range;
}

// Function to initialize the nodes with random positions and packets
void init_nodes()
{
    for (int i = 0; i < num_nodes; i++)
    {
        Node n;
        n.id = i;
        n.x = rand_range(0, net_area);
        n.y = rand_range(0, net_area);
        n.has_pkt = true;
        n.dest_id = rand_range(0, num_nodes);
        while (n.dest_id == n.id)
        {
            n.dest_id = rand_range(0, num_nodes);
        }
        n.wait_time = rand_range(0, cw_min);
        n.backoff_count = 0;
        n.retrans_count = 0;
        n.is_sender = false;
        n.is_neighbor = false;
        n.data_slots.clear();
        nodes.push_back(n);
    }
}

// Function to create a control packet (RTS or CTS) from a node
CtrlPkt create_ctrl_pkt(Node &n, bool is_rts) {
    CtrlPkt cp;
    cp.src_id = n.id;
    cp.dest_id = n.dest_id;
    cp.is_rts = is_rts;
    cp.is_fd = true;
    cp.ao = 0;
    cp.reserved_slots.clear();

    if (is_rts) {
        for (int i = 0; i < num_slots; i++) {
            if (n.data_slots[i] == 0) {
                cp.reserved_slots.push_back(i);
                n.data_slots[i] = n.id;
                cp.ao++;
            }

            if (cp.ao == 4) {
                break;
            }
        }
    } else {
        // Создать новый объект CtrlPkt из последнего элемента ctrl_pkts, который является объектом Message
        CtrlPkt last_cp(ctrl_pkts.back());
        cp.reserved_slots = last_cp.reserved_slots; // Скопировать поле reserved_slots из last_cp
        cp.ao = last_cp.ao; // Скопировать поле ao из last_cp
    }

    return cp;
}



// Function to create a data packet from a node
DataPkt create_data_pkt(Node & n)
{
    DataPkt dp;
    dp.src_id = n.id;
    dp.dest_id = n.dest_id;
    dp.payload = rand_payload(pkt_len); // generate a random payload
    return dp;
}

// Function to check if a control packet is valid (no collision or interference)
bool is_valid_ctrl_pkt(CtrlPkt & cp)
{
	std::vector<CtrlPkt> ctrl_pkt_vector; // Create a new vector of type std::vector<CtrlPkt>
	for (Message &m: ctrl_pkts) // Loop through the vector of type std::vector<Message>
	{
		CtrlPkt cp(m, num_slots); // Create a CtrlPkt object from a Message object using the second constructor
		ctrl_pkt_vector.push_back(cp); // Add the CtrlPkt object to the new vector
	}
	for (CtrlPkt &other_cp: ctrl_pkt_vector) // Loop through the new vector of type std::vector<CtrlPkt>
	{
		if (other_cp.src_id != cp.src_id && other_cp.dest_id != cp.dest_id)
		{
			// check if the control packets are from different senders or receivers
			Node &src1 = nodes[cp.src_id];
			Node &dest1 = nodes[cp.dest_id];
			Node &src2 = nodes[other_cp.src_id];
			Node &dest2 = nodes[other_cp.dest_id];
			if (is_in_range(src1, src2) || is_in_range(src1, dest2) || is_in_range(dest1, src2) || is_in_range(dest1, dest2))
			{
			 	// check if the nodes are within range of each other
				return false;	// control packet is invalid due to collision or interference
			}
		}
	}

	return true;	// control packet is valid
}	// this is the closing bracket for is_valid_ctrl_pkt


// Function to exchange messages between the nodes based on the graph and the control slots
void exchange_messages()
{
	// Create a vector of vectors to store the adjacency matrix of the graph
	vector<vector < int>> adj_matrix(nodes.size(), vector<int> (nodes.size(), 0));

	// Loop over the edges_t of the graph and update the adjacency matrix
	/*for (int i = 0; i < Edge.size(); i++) { 
        cfd e = Edge[i]; 
        adj_matrix[e.src][e.dest] = 1; 
        adj_matrix[e.dest][e.src] = 1; }*/

	// Loop over the control slots and check their values
	for (int i = 0; i < control_slots.size(); i++)
	{
		if (control_slots[i] == 1)
		{
			// Check if the control slot is reserved for a message

			// Find the source and destination nodes of the message
			Node *src = nullptr;
			Node *dest = nullptr;

			// Loop over the nodes and check their status
			for (auto &n: nodes)
			{
				if (n.status == SENDING && n.slot == i)
				{
				 		// The node is sending a message in this slot
					src = &n;
				}

				if (n.status == RECEIVING && n.slot == i)
				{
				 		// The node is receiving a message in this slot
					dest = &n;
				}
			}

			// Check if the source and destination nodes are found and valid
			if (src != nullptr && dest != nullptr && adj_matrix[src->id][dest->id] == 1)
			{
			 	// The nodes are connected by an edge in the graph

				// Create a message object with the source and destination ids and a random payload
				Message msg(src->id, dest->id, rand() % 100);

				// Add the message to the queue of the destination node
				dest->queue.push(msg);

				// Print a log message
				cout << "Node " << src->id << " sent a message to node " << dest->id << " in slot " << i << endl;
			}
		}
	}
}	// this is the closing bracket for exchange_messages

void assign_data_slots()
{
	// Create a vector of vectors to store the adjacency matrix of the graph
	vector<vector < int>> adj_matrix(nodes.size(), vector<int> (nodes.size(), 0));

	// Loop over the edges_t of the graph and update the adjacency matrix
	/* for (cfd &e: Edge)
	{
		adj_matrix[e.src_id][e.dest_id] = 1;
		adj_matrix[e.dest_id][e.src_id] = 1;
	}*/

	// Create a vector of vectors to store the table that shows which nodes can exchange messages in one slot without collisions
	vector<vector < int>> table(nodes.size(), vector<int> (nodes.size(), 0));

	// Loop over the nodes and update the table
	for (Node &n: nodes)
	{
		// Loop over the other nodes and check their distances from the current node
		for (Node &m: nodes)
		{
			if (n.id != m.id)
			{
			 	// Check if the nodes are different

				// Calculate the distance between the nodes
				double dist = sqrt(pow(n.x - m.x, 2) + pow(n.y - m.y, 2));

				if (dist <= n.trans_range && dist <= m.trans_range)
				{
				 		// Check if the nodes are within each other's transmission range

					// Set the table value to 1
					table[n.id][m.id] = 1;
				}
			}
		}
	}

	// Create a vector of sets to store the data slots for each node
	vector<set < int>> data_slots(nodes.size());

	// Create a set to store the used data slots
	set<int> used_slots;

	// Create a variable to store the current data slot
	int curr_slot = 0;

	// Loop over the nodes and assign data slots to them
	for (Node &n: nodes)
	{
		// Check if the node has a packet to send
		if (n.has_pkt)
		{
			// Loop over the other nodes and check their data slots
			for (Node &m: nodes)
			{
				if (n.id != m.id)
				{
				 		// Check if the nodes are different

					// Check if the nodes are adjacent in the graph
					if (adj_matrix[n.id][m.id] == 1)
					{
					 			// Check if the nodes can exchange messages in one slot without collisions
						if (table[n.id][m.id] == 1)
						{
						 				// Add the data slots of the other node to the used slots set
							used_slots.insert(data_slots[m.id].begin(), data_slots[m.id].end());
						}
					}
				}
			}

			// Find the first available data slot for the current node
			while (used_slots.count(curr_slot) > 0)
			{
				curr_slot++;
			}

			// Assign the current data slot to the current node
			data_slots[n.id].insert(curr_slot);

			// Clear the used slots set
			used_slots.clear();
		}
	}

	// Loop over the nodes and update their data slots vector
	for (Node &n: nodes)
	{
		// Resize the data slots vector to match the number of data slots assigned to the node
		n.data_slots.resize(data_slots[n.id].size());

		// Copy the data slots from the set to the vector
		copy(data_slots[n.id].begin(), data_slots[n.id].end(), n.data_slots.begin());

		// Print a log message
		cout << "Node " << n.id << " has " << n.data_slots.size() << " data slot(s): ";

		// Loop over the data slots vector and print its values
		for (int i = 0; i < n.data_slots.size(); i++)
		{
			cout << n.data_slots[i] << " ";
		}

		cout << endl;
	}
}


// Check if the source node has a packet to send
if (src->has_pkt)
{
	// Set the message type to FD-RTS
	msg.type = "FD-RTS";

	// Set the message data to the packet data
	msg.data = src->pkt_data;

	// Send the message from the source node to the destination node
	send_message(msg);

	// Print a log message
	cout << "Node " << src->id << " sent FD-RTS to Node " << dest->id << endl;
}

if (control_slots[i] == 0)
{
	// Check if the control slot is empty

	// Do nothing
}

if (control_slots[i] == -1)
{
	// Check if the control slot is reserved for a collision

	// Find the nodes that are involved in the collision
	vector<Node*> colliding_nodes;

	// Loop over the nodes and check their control slots
	for (Node &n: nodes)
	{
		if (n.control_slots[i] != "")
		{
			// Check if the node is sending a message in this slot

			// Add the node to the vector of colliding nodes
			colliding_nodes.push_back(&n);
		}
	}

	if (!colliding_nodes.empty())
	{
		// Check if there are some colliding nodes

		// Create a message object with no source and destination nodes
		Message msg(nullptr, nullptr);

		// Set the message type to COLLISION
		msg.type = "COLLISION";

		// Set the message data to an empty string
		msg.data = "";

		// Send the message to all colliding nodes
		for (Node *n: colliding_nodes)
		{
			send_message(msg, n);

			// Print a log message
			cout << "Node " << n->id << " detected a collision in slot " << i << endl;
		}
	}
}


return true;	// control packet is valid


// Function to check if a data packet is valid (no collision or interference)
bool is_valid_data_pkt(DataPkt & dp)
{
	for (DataPkt &other_dp: data_pkts)
	{
		if (other_dp.src_id != dp.src_id && other_dp.dest_id != dp.dest_id)
		{
			// check if the data packets are from different senders or receivers
			Node &src1 = nodes[dp.src_id];
			Node &dest1 = nodes[dp.dest_id];
			Node &src2 = nodes[other_dp.src_id];
			Node &dest2 = nodes[other_dp.dest_id];
			if (is_in_range(src1, src2) || is_in_range(src1, dest2) || is_in_range(dest1, src2) || is_in_range(dest1, dest2))
			{
			 	// check if the nodes are within range of each other
				return false;	// data packet is invalid due to collision or interference
			}
		}
	}

	return true;	// data packet is valid
}

// Function to create a control packet
CtrlPkt create_ctrl_pkt(Node &n, bool is_rts)
{
	CtrlPkt cp;	// create a control packet object
	cp.src_id = n.id;	// set the source ID
	cp.dest_id = n.dest_id;	// set the destination ID
	cp.is_rts = is_rts;	// set the RTS flag
	cp.is_fd = true;	// set the FD flag
	cp.ao = 0;	// initialize the AO value

	if (is_rts)
	{
		// if the packet is RTS

		cp.reserved_slots.clear();	// clear the reserved slots vector

		for (int i = 10; i < 15; i++)
		{
			// loop over the data slots

			if (data_slots[i] == 0)
			{
			 	// if the data slot is free

				cp.reserved_slots.push_back(i);	// add the data slot to the reserved slots vector
				data_slots[i] = n.id;	// mark the data slot as reserved by the node ID
				cp.ao++;	// increment the AO value
			}

			if (cp.ao == 4)
			{
			 	// if the AO value reaches 4

				break;	// break the loop
			}
		}
	}
	else
	{
		// if the packet is CTS

		cp.reserved_slots = ctrl_pkts.back().reserved_slots;	// copy the reserved slots vector from the last RTS packet
		cp.ao = ctrl_pkts.back().ao;	// copy the AO value from the last RTS packet
	}

	return cp;	// return the control packet object
}

// Simulate one frame of CFFD-MAC protocol
void simulate_frame(int num_nodes, int num_slots)
{
    // Initialize the control slots vector with zeros
    std::vector<int> control_slots(num_slots, 0);
    // Initialize the data slots vector with zeros
    std::vector<int> data_slots(num_slots, 0);
    // Initialize the conflict slots vector with zeros
    std::vector<int> conflict_slots(num_slots, 0);
    // Initialize the reserved slots vector with zeros
    std::vector<int> reserved_slots(num_slots, 0);

    // Control phase
    for (int i = 0; i < num_nodes; i++)
    {
        // Check if node i has a packet to send
        if (!nodes[i].queue.empty())
        {
            // Get the message from the queue
            Message msg = nodes[i].queue.front();
            // Get the destination node id
            int dest = msg.destination_id;
            // Get the random backoff time from node i
            int backoff = nodes[i].backoff;
            // Wait for DIFS time and backoff time before sending FD-RTS
            int wait_time = difs_time + backoff;
            // Check if the control slot is available after waiting
            if (control_slots[wait_time] == 0)
            {
                // Send FD-RTS to destination node and all neighbors
                CtrlPkt rts(msg, true, true, 0, num_slots);
                nodes[i].send(rts);
                // Mark the control slot as occupied by node i
                control_slots[wait_time] = i + 1;
                // Pop the message from the queue
                nodes[i].queue.pop();
                // Reset the backoff time of node i
                nodes[i].backoff = 0;
            }
            else
            {
                // Collision occurs, double the CW and choose a new backoff time
                nodes[i].CW = nodes[i].CW * 2;
                nodes[i].backoff = rand() % nodes[i].CW;
            }
        }
    }

    // Receive FD-RTS and send FD-CTS
    for (int j = 0; j < num_nodes; j++)
    {
        // Check if node j received any FD-RTS
        if (!nodes[j].rx_buffer.empty())
        {
            // Get the FD-RTS from the rx buffer
            CtrlPkt rts = nodes[j].rx_buffer.front();
            // Get the source node id
            int src = rts.msg.source_id;
            // Check if node j is the destination node or a neighbor node
            if (j == src)
            {
                // Ignore self-sent FD-RTS
                continue;
            }
            else if (j == rts.msg.destination_id)
            {
                                // Node j is the destination node, send FD-CTS to source node with AO=0 and reserved slots vector
                CtrlPkt cts(rts.msg, false, true, 0, rts.reserved_slots);
                nodes[j].send(cts);
                // Mark the data slot as reserved by node j
                data_slots[rts.AO] = j + 1;
                // Pop the FD-RTS from the rx buffer
                nodes[j].rx_buffer.pop();
            }
            else
            {
                // Node j is a neighbor node, send FD-CTS to source node with AO=X and reserved slots vector
                CtrlPkt cts(rts.msg, false, true, X, rts.reserved_slots);
                nodes[j].send(cts);
                // Mark the data slot as conflicted by node j
                conflict_slots[rts.AO] = j + 1;
                // Pop the FD-RTS from the rx buffer
                nodes[j].rx_buffer.pop();
            }
        }
    }

    // Receive FD-CTS and resolve conflicts
    for (int k = 0; k < num_nodes; k++)
    {
        // Check if node k received any FD-CTS
        if (!nodes[k].rx_buffer.empty())
        {
            // Get the FD-CTS from the rx buffer
            CtrlPkt cts = nodes[k].rx_buffer.front();
            // Get the destination node id
            int dest = cts.msg.destination_id;
            // Check if node k is the source node or a neighbor node
            if (k == dest)
            {
                // Ignore self-sent FD-CTS
                continue;
            }
            else if (k == cts.msg.source_id)
            {
                // Node k is the source node, check the AO value of FD-CTS
                if (cts.AO == 0)
                {
                    // Node k received FD-CTS from destination node, mark the data slot as available for TX/RX
                    data_slots[cts.AO] = 1;
                    // Update the reserved slots vector with the one received from destination node
                    reserved_slots = cts.reserved_slots;
                }
                else if (cts.AO == X)
                {
                    // Node k received FD-CTS from a neighbor node, mark the data slot as conflicted for TX/RX
                    conflict_slots[cts.AO] = 1;
                    // Check if there is any conflict in the reserved slots vector with the one received from neighbor node
                    for (int s = 0; s < num_slots; s++)
                    {
                        if (reserved_slots[s] != cts.reserved_slots[s])
                        {
                            // Conflict detected, mark the slot as conflicted for TX/RX
                            conflict_slots[s] = 1;
                        }
                    }
                }
                                else if (cts.AO == X)
                {
                    // Node k received FD-CTS from a neighbor node, mark the data slot as conflicted for TX/RX
                    conflict_slots[cts.AO] = 1;
                    // Check if there is any conflict in the reserved slots vector with the one received from neighbor node
                    for (int s = 0; s < num_slots; s++)
                    {
                        if (reserved_slots[s] != cts.reserved_slots[s])
                        {
                            // Conflict detected, mark the slot as conflicted for TX/RX
                            conflict_slots[s] = 1;
                        }
                    }
                }
                // Pop the FD-CTS from the rx buffer
                nodes[k].rx_buffer.pop();
            }
            else
            {
                // Node k is a neighbor node, ignore FD-CTS from other nodes
                continue;
            }
        }
    }

    // Data phase
    for (int l = 0; l < num_nodes; l++)
    {
        // Check if node l has a data slot available for TX/RX
        if (data_slots[l] == 1)
        {
            // Node l can send or receive data in slot l
            nodes[l].tx_data();
            nodes[l].rx_data();
        }
        else if (conflict_slots[l] == 1)
        {
            // Node l has a conflict in slot l, cannot send or receive data
            nodes[l].tx_conflict();
            nodes[l].rx_conflict();
        }
    }
}

// Function to print the simulation results for one frame
void print_results()
{
    cout << "Control packets:\n";
    for (CtrlPkt &cp : ctrl_pkts)
    {
        cout << "Source: " << cp.src_id << ", Destination: " << cp.dest_id << ", RTS: " << (cp.is_rts ? "Yes" : "No") << ", FD: " << (cp.is_fd ? "Yes" : "No") << ", AO: " << cp.ao << ", Reserved slots: ";
        for (int slot : cp.reserved_slots)
        {
            cout << slot << " ";
        }

        cout << "\n";
    }

    cout << "Data packets:\n";
    for (DataPkt &dp : data_pkts)
    {
        cout << "Source: " << dp.src_id << ", Destination: " << dp.dest_id << ", Data: " << dp.data << "\n";
    }

    cout << "Feedback packets:\n";
    for (FbPkt &fp : fb_pkts)
    {
        cout << "Source: " << fp.src_id << ", Destination: " << fp.dest_id << ", ACK: " << (fp.is_ack ? "Yes" : "No") << "\n";
    }
 
    cout << "Statistics:\n";
    int total_pkts = 0;
    int success_pkts = 0;
    int fail_pkts = 0;
    double success_rate = 0.0;
    double fail_rate = 0.0;

    for (Node &n : nodes)
    {
        total_pkts += n.retrans_count + 1;
        if (n.has_pkt)
        {
            fail_pkts++;
        }
        else
        {
            success_pkts++;
        }
    }

    success_rate = (double)success_pkts / total_pkts * 100;
    fail_rate = (double)fail_pkts / total_pkts * 100;

    cout << "Total packets: " << total_pkts << "\n";
    cout << "Successful packets: " << success_pkts << "\n";
    cout << "Failed packets: " << fail_pkts << "\n";
    cout << "Success rate: " << success_rate << "%\n";
    cout << "Fail rate: " << fail_rate << "%\n";
}
	cout << "Nodes:\n";
	for (Node &n: nodes)
	{
		cout << "ID: " << n.id << ", Position: (" << n.x << ", " << n.y << "), Has packet: " << (n.has_pkt ? "Yes" : "No") << ", Destination: " << n.dest_id << ", Wait time: " << n.wait_time << ", Backoff         count: " << n.backoff_count << ", Retransmission count: " << n.retrans_count << ", Is sender: " << (n.is_sender ? "Yes" : "No") << ", Is neighbor: " << (n.is_neighbor ? "Yes" : "No") << ", Data slots: ";
		for (int slot: n.data_slots)
		{
			cout << slot << " ";
		}

		cout << "\n";
	}

    // Function to print the simulation results for all frames

// Main function
int main()
{
    // Other code as needed
    Edge.push_back(cfd{&node1, &node2}); // Add an edge from node1 to node2 to the vector
    Edge.push_back(cfd{&node2, &node3}); // Add another edge from node2 to node3 to the vector
    srand(time(NULL));
    init_nodes();
    for (int i = 0; i < 100; i++)
    {
        cout << "Frame " << i + 1 << ":\n";
		generate_pkts();	// generate packets for the nodes
		simulate_frame();	// simulate the CFFD-MAC protocol for one frame
		print_results();	// print the simulation results for one frame
		update_nodes();	// update the nodes' states
		clear_pkts();	// clear the packets' vectors
	}

	return 0;
};
