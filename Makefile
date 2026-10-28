CXX      := g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Werror -pthread -Iinclude
TARGET   := waste_sorter

all: $(TARGET)

$(TARGET): src/ishitva_suka_waste_sorter.cpp include/sorting_engine.hpp
	$(CXX) $(CXXFLAGS) src/ishitva_suka_waste_sorter.cpp -o $(TARGET)

test: $(TARGET)
	./$(TARGET) --test

simulate: $(TARGET)
	./$(TARGET) --simulate

clean:
	rm -f $(TARGET)

.PHONY: all test simulate clean
