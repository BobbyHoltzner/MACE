addpath ..
startup
TCPServer(@(x)x+1, 'port', 1234, 'onetime', true);
