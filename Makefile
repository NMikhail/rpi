CC = gcc

hameleon:
	$(CC) hameleon.c mcp2515.c ultrasound.c cc1101_func.c -o hameleon -lwiringPi -lpthread
example_MCP: 
	$(CC) example_MCP.c mcp2515.c -o example_MCP -lwiringPi
