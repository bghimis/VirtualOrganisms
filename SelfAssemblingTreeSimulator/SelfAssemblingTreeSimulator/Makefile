CXX=g++
CXXFLAGS=-g -std=c++11
MKDIR_P=mkdir -p
ODIR=../Release/obj
RDIR=../Release
DEPS=Utils.h Cell.h BoardObject.h SimulatorBoard.h ExprGenerator.h 

_OBJ=Utils.o Cell.o BoardObject.o SimulatorBoard.o 
OBJ=$(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.cpp $(DEPS)
	$(MKDIR_P) $(ODIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $< 
$(RDIR)/program: $(OBJ) main.cpp
	$(CXX) $(CXXFLAGS) -o $@ $^ 

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o $(RDIR)/program
