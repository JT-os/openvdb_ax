///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015-2020 DNEG
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of DNEG nor the names
// of its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////

#include <openvdb_ax/ast/AST.h>
#include <openvdb_ax/ast/Scanners.h>
#include <openvdb_ax/ast/PrintTree.h>
#include <openvdb_ax/Exceptions.h>
#include <openvdb_ax/test/util.h>

#include <cppunit/extensions/HelperMacros.h>

#include <string>

using namespace openvdb::ax::ast;
using namespace openvdb::ax::ast::tokens;

namespace {

static const unittest_util::CodeTests tests =
{
    { "-a;",            Node::Ptr(new UnaryOperator(OperatorToken::MINUS, new Local("a"))) },
    { "+a;",            Node::Ptr(new UnaryOperator(OperatorToken::PLUS, new Local("a"))) },
    { "!a;",            Node::Ptr(new UnaryOperator(OperatorToken::NOT, new Local("a"))) },
    { "~a;",            Node::Ptr(new UnaryOperator(OperatorToken::BITNOT, new Local("a"))) },
    { "~~a;",           Node::Ptr(new UnaryOperator(OperatorToken::BITNOT, new UnaryOperator(OperatorToken::BITNOT, new Local("a")))) },
    { "!~a;",           Node::Ptr(new UnaryOperator(OperatorToken::NOT, new UnaryOperator(OperatorToken::BITNOT, new Local("a")))) },
    { "+-a;",           Node::Ptr(new UnaryOperator(OperatorToken::PLUS, new UnaryOperator(OperatorToken::MINUS, new Local("a")))) },
    { "-+a;",           Node::Ptr(new UnaryOperator(OperatorToken::MINUS, new UnaryOperator(OperatorToken::PLUS, new Local("a")))) },
    { "!!!a;",          Node::Ptr(new UnaryOperator(OperatorToken::NOT,
                            new UnaryOperator(OperatorToken::NOT,
                                new UnaryOperator(OperatorToken::NOT, new Local("a"))
                            )
                        ))
    },
    { "~~~a;",          Node::Ptr(new UnaryOperator(OperatorToken::BITNOT,
                            new UnaryOperator(OperatorToken::BITNOT,
                                new UnaryOperator(OperatorToken::BITNOT, new Local("a"))
                            )
                        ))
    },
    { "-(a+b);",        Node::Ptr(new UnaryOperator(OperatorToken::MINUS,
                            new BinaryOperator(OperatorToken::PLUS,
                                new Local("a"), new Local("b")
                            )
                        ))
    },
    { "!func();",       Node::Ptr(new UnaryOperator(OperatorToken::NOT, new FunctionCall("func"))) },
    { "-@a;",           Node::Ptr(new UnaryOperator(OperatorToken::MINUS, new Attribute("a", CoreType::FLOAT, true))) },
    { "!v@a;",          Node::Ptr(new UnaryOperator(OperatorToken::NOT, new Attribute("a", CoreType::VEC3F))) },
    { "~v@a;",          Node::Ptr(new UnaryOperator(OperatorToken::BITNOT, new Attribute("a", CoreType::VEC3F))) },
    { "+int(a);",       Node::Ptr(new UnaryOperator(OperatorToken::PLUS, new Cast(new Local("a"), CoreType::INT))) },
    { "-(float(a));",   Node::Ptr(new UnaryOperator(OperatorToken::MINUS, new Cast(new Local("a"), CoreType::FLOAT))) },
    { "!a.x;",          Node::Ptr(new UnaryOperator(OperatorToken::NOT, new ArrayUnpack(new Local("a"), new Value<int32_t>(0)))) },
    { "-a[0];",         Node::Ptr(new UnaryOperator(OperatorToken::MINUS, new ArrayUnpack(new Local("a"), new Value<int32_t>(0)))) },
    { "-++a;",          Node::Ptr(new UnaryOperator(OperatorToken::MINUS, new Crement(new Local("a"), Crement::Operation::Increment, false))) },
    { "!{a,b,c};",      Node::Ptr(new UnaryOperator(OperatorToken::NOT,
                            new ArrayPack(new ExpressionList({
                                new Local("a"),
                                new Local("b"),
                                new Local("c")
                            }))
                        ))
    },
    // This is a bit of a weird one - should perhaps look to making this a syntax error
    // (it will fail at compilation with an lvalue error)
    { "-a=a;",          Node::Ptr(new UnaryOperator(OperatorToken::MINUS,
                            new AssignExpression(new Local("a"), new Local("a"), false)
                        ))
    }
};

}

class TestUnaryOperatorNode : public CppUnit::TestCase
{
public:

    CPPUNIT_TEST_SUITE(TestUnaryOperatorNode);
    CPPUNIT_TEST(testSyntax);
    CPPUNIT_TEST(testASTNode);
    CPPUNIT_TEST_SUITE_END();

    void testSyntax() { TEST_SYNTAX_PASSES(tests); }
    void testASTNode();
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestUnaryOperatorNode);

void TestUnaryOperatorNode::testASTNode()
{
    for (const auto& test : tests) {
        const std::string& code = test.first;
        const Node* expected = test.second.get();
        const Tree::Ptr tree = parse(code.c_str());
        CPPUNIT_ASSERT_MESSAGE(ERROR_MSG("No AST returned", code), static_cast<bool>(tree));

        // get the first statement
        const Node* result = tree->child(0)->child(0);
        CPPUNIT_ASSERT(result);
        CPPUNIT_ASSERT_MESSAGE(ERROR_MSG("Invalid AST node", code),
            Node::UnaryOperatorNode == result->nodetype());

        std::vector<const Node*> resultList, expectedList;
        linearize(*result, resultList);
        linearize(*expected, expectedList);

        if (!unittest_util::compareLinearTrees(expectedList, resultList)) {
            std::ostringstream os;
            os << "\nExpected:\n";
            openvdb::ax::ast::print(*expected, true, os);
            os << "Result:\n";
            openvdb::ax::ast::print(*result, true, os);
            CPPUNIT_FAIL(ERROR_MSG("Mismatching Trees for Unary Operator code", code) + os.str());
        }
    }
}

// Copyright (c) 2015-2020 DNEG
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
