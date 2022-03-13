#include "llvm/Support/raw_ostream.h"

#include <string>
#include <iostream>
#include <system_error>

#include "ast.hpp"
#include "parser.hpp"

using namespace llvm;

inline llvm::raw_ostream& operator<<(llvm::raw_ostream& os,
    const ast::Node& ast) {
    os << ast.to_string();
    return os;
}


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cout << "Usage: ./code InputFile\n";
        return 1;
    }

    bool flag = parser::registerFile(argv[1]);
    if (flag) {
        perror("Error opening file.");
    }

    //Run Parser
    parser::getNextToken();
    fprintf(stderr, "Parsing Source Code...\n");
    auto program = parser::ParseProgram();
    fprintf(stderr, "Parsing Finished!\n");

    if (!program) {
        parser::closeFile();
        return 0;
    }

    //Print out AST
    fprintf(stderr, "Printing AST...\n");
    llvm::outs() << program.get()->to_string();
    fprintf(stderr, "AST Printed!\n");

    //Run Evaluation Step

    parser::closeFile();
    return 0;
}