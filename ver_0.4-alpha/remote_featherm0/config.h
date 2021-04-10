#define this_node_id 4 // remote node must have node_id > 1 (1 is the gateway)

const int interval_sec = 300;
const int random_interval_sec = 3; // range of random seconds to add to interval_sec
const int forcePPM = 400;

// don't need to modify this unless you have more than one gateway within lora range
const char* loranet_pubkey = "zgpqdys5a9r3"; //this will be 'network A'; we can add more networks later if need be
