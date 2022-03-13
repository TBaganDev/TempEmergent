#pragma once

#include <string.h>
#include <string>

namespace lexer {
	enum TOKEN_TYPE {
		INVALID = -100,
		END_OF_FILE = 0,
		IDENTIFIER = -1,
		LBRA = int('{'),  // Left Brace
		RBRA = int('}'),  // Right Brace
		LPAR = int('('),  // Left Parenthesis
		RPAR = int(')'),  // Right Parenthesis
		COMMA = int(','), // Comma
		COLON = int(':'),
		// Logical Operators
		AND = int('&'),
		OR = int('|'),
		NOT = int('!'),
		// Comparison Operators
		EQ = -2,
		NEQ = -3
	};

	struct TOKEN {
		TOKEN_TYPE type = INVALID;
		int line_number;
		int column_number;
		std::string lexeme;
	};

	void resetLexer();
	static TOKEN makeToken(std::string lexeme, TOKEN_TYPE tok_type);
	TOKEN getTok(FILE* filename);
}