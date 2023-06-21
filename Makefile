# usage: make

# compilatore
CC = gcc

# nome directory principali
BIN_DIR = bin
INCL_DIR = include
SRC_DIR = src
OBJ_DIR = obj

# flag compilatore
INCL_FLAG = $(addprefix -I,$(INCL_DIR))
CFLAGS = $(INCL_FLAG) -MMD -MP -g

# flag linker
LINK_FLAG = -lrt

# path file sorgente
SRCS := $(shell find $(SRC_DIR) -name '*.c')
# path file oggetto
OBJS := $(SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
# path file dipendenze
DEPS := $(OBJS:.o=.d)

# nome file eseguibili
INPUT_BIN = input
OUTPUT_BIN = output

# target senza regole
.PHONY: clean

$(BIN_DIR)/input: $(OBJ_DIR)/input.o $(OBJ_DIR)/error.o $(OBJ_DIR)/ecu_connector.o
	mkdir -p $(dir $@)
	$(CC) $< -o $@ $(LINK_FLAG)

$(BIN_DIR)/output: $(OBJ_DIR)/output.o $(OBJ_DIR)/error.o $(OBJ_DIR)/ecu_connector.o
	mkdir -p $(dir $@)
	$(CC) $< -o $@ $(LINK_FLAG)

# compilazione file sorgente
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# usage: make clean
# eliminazione file: oggetto, eseguibili, dipendenze, log e segmento
clean:
	rm -rf bin obj log &> /dev/null

-include $(DEPS)