Header header
int16 number
string function_name
string component_name
string parameter_name
string full_graph_name

# Possible execution states
byte UNKNOWN=0
byte ACTIVE=1
byte SUCCEEDED=2
byte FAILED=3

byte state # state of execution
int16 error_code # current error_code for state
