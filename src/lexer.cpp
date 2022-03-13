#include "lexer.hpp"

using namespace lexer;

static std::string text;
static int line_number, column_number;

void lexer::resetLexer() {
	line_number = 1;
	column_number = 1;
}

lexer::TOKEN lexer::makeToken(std::string lexeme, lexer::TOKEN_TYPE tok_type) {
	lexer::TOKEN token;
	return_tok.lexeme = lexeme;
	return_tok.type = tok_type;
	return_tok.line_number = line_number;
	return_tok.column_number = column_number - lexeme.length() - 1;
	return token;
}

lexer::TOKEN lexer::getTok(FILE* source) {
	static int cursor = ' ';
	static int cursor_lookahead = ' ';

	//Ignores whitespace
	while (isspace(last_char)) {
		if (cursor == '\n' || cursor == '\r') {
			line_number++;
			column_number = 1; //As it is a new line, cursor at the beginning
		}
		cursor = getc(source);
		column_number++;
	}

	column_number++;

	if (cursor == EOF) {
		return makeToken("EOF", END_OF_FILE);
	}

	if (cursor == '=') {
		cursor = getc(source);
		if (cursor == '=') {
			column_number++;
			return makeToken("==", EQ);
		}
		else {
			return makeToken('Expected comparison operator \'==\'', INVALID);
		}
	}

	if (cursor == '!') {
		cursor_lookahead = getc(source);
		if (cursor_lookahead == '=') {
			cursor = getc(source);
			column_number++;
			return makeToken("!=", NEQ);
		} else {
			cursor = cursor_lookahead; //Move cursor forward
			return makeToken("!", NOT)
		}
	}


	std::string lex(1, cursor); //Converts (char) int cursor into string
	lexer::TOKEN_TYPE type = int(cursor);
	cursor = getc(source);
	return makeToken(lex, type);
}