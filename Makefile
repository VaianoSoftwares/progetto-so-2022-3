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
LINK_FLAG_FRONT = -Wall -Werror
LINK_FLAG_BACK = -lrt

# path file sorgente
SRCS := $(shell find $(SRC_DIR) -name '*.c')
# path file oggetto
OBJS := $(SRCS:$(SRC_DIR)/%.c=$(OBJ_DIR)/%.o)
# path file dipendenze
DEPS := $(OBJS:.o=.d)

# lista oggetti per linking
_INPUT_OBJS = input error ecu_connector
INPUT_OBJS = $(_INPUT_OBJS:%=$(OBJ_DIR)/%.o)
_OUTPUT_OBJS = output error ecu_connector
OUTPUT_OBJS = $(_OUTPUT_OBJS:%=$(OBJ_DIR)/%.o)

# nome file eseguibili
INPUT_BIN = input
OUTPUT_BIN = output

# target senza regole
.PHONY: clean

# usage: make
# creazione file eseguibili
all: $(BIN_DIR)/$(INPUT_BIN) $(BIN_DIR)/$(OUTPUT_BIN)

# creazione file eseguibile input
$(BIN_DIR)/$(INPUT_BIN): $(INPUT_OBJS)
	mkdir -p $(dir $@)
	$(CC) $(LINK_FLAG_FRONT) $(INPUT_OBJS) -o $@ $(LINK_FLAG_BACK)

# creazione file eseguibile output
$(BIN_DIR)/$(OUTPUT_BIN): $(OUTPUT_OBJS)
	mkdir -p $(dir $@)
	$(CC) $(LINK_FLAG_FRONT) $(OUTPUT_OBJS) -o $@ $(LINK_FLAG_BACK)

# compilazione file sorgente
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

# usage: make clean
# eliminazione file: oggetto, eseguibili, dipendenze, log e segmento
clean:
	rm -rf bin obj log &> /dev/null

-include $(DEPS)