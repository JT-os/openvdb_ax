/* A Bison parser, made by GNU Bison 3.0.5.  */

/* Bison interface for Yacc-like parsers in C

   Copyright (C) 1984, 1989-1990, 2000-2015, 2018 Free Software Foundation, Inc.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.  */

/* As a special exception, you may create a larger work that contains
   part or all of the Bison parser skeleton and distribute that work
   under terms of your choice, so long as that work isn't itself a
   parser generator using the skeleton or a modified version thereof
   as a parser skeleton.  Alternatively, if you modify or redistribute
   the parser skeleton itself, you may (at your option) remove this
   special exception, which will cause the skeleton and the resulting
   Bison output files to be licensed under the GNU General Public
   License without this special exception.

   This special exception was added by the Free Software Foundation in
   version 2.2 of Bison.  */

#ifndef YY_AX_OPENVDB_AX_GRAMMAR_AXPARSER_H_INCLUDED
# define YY_AX_OPENVDB_AX_GRAMMAR_AXPARSER_H_INCLUDED
/* Debug traces.  */
#ifndef AXDEBUG
# if defined YYDEBUG
#if YYDEBUG
#   define AXDEBUG 1
#  else
#   define AXDEBUG 0
#  endif
# else /* ! defined YYDEBUG */
#  define AXDEBUG 0
# endif /* ! defined YYDEBUG */
#endif  /* ! defined AXDEBUG */
#if AXDEBUG
extern int axdebug;
#endif

/* Token type.  */
#ifndef AXTOKENTYPE
# define AXTOKENTYPE
  enum axtokentype
  {
    TRUE = 258,
    FALSE = 259,
    SEMICOLON = 260,
    AT = 261,
    DOLLAR = 262,
    IF = 263,
    ELSE = 264,
    FOR = 265,
    DO = 266,
    WHILE = 267,
    RETURN = 268,
    BREAK = 269,
    CONTINUE = 270,
    LCURLY = 271,
    RCURLY = 272,
    LSQUARE = 273,
    RSQUARE = 274,
    STRING = 275,
    DOUBLE = 276,
    FLOAT = 277,
    LONG = 278,
    INT = 279,
    SHORT = 280,
    BOOL = 281,
    VOID = 282,
    VEC2I = 283,
    VEC2F = 284,
    VEC2D = 285,
    VEC3I = 286,
    VEC3F = 287,
    VEC3D = 288,
    VEC4I = 289,
    VEC4F = 290,
    VEC4D = 291,
    F_AT = 292,
    I_AT = 293,
    V_AT = 294,
    S_AT = 295,
    MAT3F = 296,
    MAT3D = 297,
    MAT4F = 298,
    MAT4D = 299,
    M3F_AT = 300,
    M4F_AT = 301,
    F_DOLLAR = 302,
    I_DOLLAR = 303,
    V_DOLLAR = 304,
    S_DOLLAR = 305,
    DOT_X = 306,
    DOT_Y = 307,
    DOT_Z = 308,
    L_SHORT = 309,
    L_INT = 310,
    L_LONG = 311,
    L_FLOAT = 312,
    L_DOUBLE = 313,
    L_STRING = 314,
    IDENTIFIER = 315,
    COMMA = 316,
    EQUALS = 317,
    PLUSEQUALS = 318,
    MINUSEQUALS = 319,
    MULTIPLYEQUALS = 320,
    DIVIDEEQUALS = 321,
    MODULOEQUALS = 322,
    BITANDEQUALS = 323,
    BITXOREQUALS = 324,
    BITOREQUALS = 325,
    OR = 326,
    AND = 327,
    BITOR = 328,
    BITXOR = 329,
    BITAND = 330,
    EQUALSEQUALS = 331,
    NOTEQUALS = 332,
    MORETHAN = 333,
    LESSTHAN = 334,
    MORETHANOREQUAL = 335,
    LESSTHANOREQUAL = 336,
    PLUS = 337,
    MINUS = 338,
    MULTIPLY = 339,
    DIVIDE = 340,
    MODULO = 341,
    NOT = 342,
    BITNOT = 343,
    PLUSPLUS = 344,
    MINUSMINUS = 345,
    LPARENS = 346,
    RPARENS = 347,
    LOWER_THAN_ELSE = 348
  };
#endif

/* Value type.  */
#if ! defined AXSTYPE && ! defined AXSTYPE_IS_DECLARED

union AXSTYPE
{


    const char* string;
    uint64_t index;

    openvdb::ax::ast::Tree* tree;
    openvdb::ax::ast::ValueBase* value;
    openvdb::ax::ast::Statement* statement;
    openvdb::ax::ast::StatementList* statementlist;
    openvdb::ax::ast::Block* block;
    openvdb::ax::ast::Expression* expression;
    openvdb::ax::ast::ExpressionList* expressionlist;
    openvdb::ax::ast::Variable* variable;
    openvdb::ax::ast::ExternalVariable* external;
    openvdb::ax::ast::Attribute* attribute;
    openvdb::ax::ast::DeclareLocal* declare_local;
    openvdb::ax::ast::Local* local;


};

typedef union AXSTYPE AXSTYPE;
# define AXSTYPE_IS_TRIVIAL 1
# define AXSTYPE_IS_DECLARED 1
#endif

/* Location type.  */
#if ! defined AXLTYPE && ! defined AXLTYPE_IS_DECLARED
typedef struct AXLTYPE AXLTYPE;
struct AXLTYPE
{
  int first_line;
  int first_column;
  int last_line;
  int last_column;
};
# define AXLTYPE_IS_DECLARED 1
# define AXLTYPE_IS_TRIVIAL 1
#endif


extern AXSTYPE axlval;
extern AXLTYPE axlloc;
int axparse (openvdb::ax::ast::Tree** tree);

#endif /* !YY_AX_OPENVDB_AX_GRAMMAR_AXPARSER_H_INCLUDED  */
