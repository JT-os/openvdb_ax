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

#include "TestHarness.h"
#include "util.h"

#include <cppunit/extensions/HelperMacros.h>


class TestUnary : public unittest_util::AXTestCase
{
public:
    CPPUNIT_TEST_SUITE(TestUnary);
    CPPUNIT_TEST(testBitwiseNot);
    CPPUNIT_TEST(testNegate);
    CPPUNIT_TEST(testNot);
    CPPUNIT_TEST(testUnaryVector);
    CPPUNIT_TEST_SUITE_END();

    void testBitwiseNot();
    void testNegate();
    void testNot();
    void testUnaryVector();
};

CPPUNIT_TEST_SUITE_REGISTRATION(TestUnary);

void
TestUnary::testBitwiseNot()
{
    mHarness.addAttributes<int>({"int_test", "int_test2"}, {-9, -8});

    mHarness.executeCode("test/snippets/unary/unaryBitwiseNot");

    AXTESTS_STANDARD_ASSERT();
}

void
TestUnary::testNegate()
{
    mHarness.addAttribute<int>("int_test", -3);
    mHarness.addAttribute<float>("float_test", -5.5f);

    mHarness.executeCode("test/snippets/unary/unaryNegate");

    AXTESTS_STANDARD_ASSERT();
}

void
TestUnary::testNot()
{
    mHarness.addAttributes<bool>({"bool_test", "bool_test2"}, {false, true});

    mHarness.executeCode("test/snippets/unary/unaryNot");

    AXTESTS_STANDARD_ASSERT();
}

void
TestUnary::testUnaryVector()
{
    // vec3

    mHarness.addAttributes<openvdb::math::Vec3<int32_t>>
        (unittest_util::nameSequence("v3i", 4), {
            openvdb::math::Vec3<int32_t>(0, 1,-1),
            openvdb::math::Vec3<int32_t>(0,-1, 1),
            openvdb::math::Vec3<int32_t>(-1,-2,0),
            openvdb::math::Vec3<int32_t>(1, 0, 0)
        });

    mHarness.addAttributes<openvdb::math::Vec3<float>>
        (unittest_util::nameSequence("v3f", 2), {
            openvdb::math::Vec3<float>(0.0f, 1.1f,-1.1f),
            openvdb::math::Vec3<float>(0.0f,-1.1f, 1.1f),
        });

    mHarness.addAttributes<openvdb::math::Vec3<double>>
        (unittest_util::nameSequence("v3d", 2), {
            openvdb::math::Vec3<double>(0.0, 1.1,-1.1),
            openvdb::math::Vec3<double>(0.0,-1.1, 1.1),
        });

    // vec4

    mHarness.addAttributes<openvdb::math::Vec4<int32_t>>
        (unittest_util::nameSequence("v4i", 4), {
            openvdb::math::Vec4<int32_t>(0, 1,-1, 2),
            openvdb::math::Vec4<int32_t>(0,-1, 1, -2),
            openvdb::math::Vec4<int32_t>(-1,-2,0,-3),
            openvdb::math::Vec4<int32_t>(1, 0, 0, 0)
        });

    mHarness.addAttributes<openvdb::math::Vec4<float>>
        (unittest_util::nameSequence("v4f", 2), {
            openvdb::math::Vec4<float>(0.0f, 1.1f,-1.1f, 2.1f),
            openvdb::math::Vec4<float>(0.0f,-1.1f, 1.1f, -2.1f)
        });

    mHarness.addAttributes<openvdb::math::Vec4<double>>
        (unittest_util::nameSequence("v4d", 2), {
            openvdb::math::Vec4<double>(0.0, 1.1,-1.1, 2.1),
            openvdb::math::Vec4<double>(0.0,-1.1, 1.1, -2.1)
        });

    mHarness.executeCode("test/snippets/unary/unaryVector");

    AXTESTS_STANDARD_ASSERT();
}

// Copyright (c) 2015-2020 DNEG
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
