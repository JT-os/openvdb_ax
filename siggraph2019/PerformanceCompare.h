///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2015-2019 DNEG
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

#ifndef OPENVDB_AX_CMD_PERFORMANCECOMPARE_HAS_BEEN_INCLUDED
#define OPENVDB_AX_CMD_PERFORMANCECOMPARE_HAS_BEEN_INCLUDED

#include <openvdb/util/CpuTimer.h>
#include <openvdb/points/PointDataGrid.h>
#include <openvdb_ax/compiler/Compiler.h>
#include <openvdb_ax/compiler/PointExecutable.h>

#include <iostream>
#include <functional>

#include <ctime>

namespace openvdb {
OPENVDB_USE_VERSION_NAMESPACE
namespace OPENVDB_VERSION_NAME {

class ComparisonCase
{
public:
    ComparisonCase(const std::string& caseName,
                   const openvdb::points::PointDataGrid::Ptr& grid,
                   const std::string& snippet,
                   std::function<void(openvdb::points::PointDataGrid::Ptr)> cppCode)
        : mCaseName(caseName)
        , mCode(snippet)
        , mGrid(grid)
        , mCppFun(cppCode)
        , mCompiler()
        , mCppDuration()
        , mAXDuration() {
            mCompiler = openvdb::ax::Compiler::create();
        }

    void execute()
    {
        // deep copy grid so that the snippet version can be executed on an unmodified input

        openvdb::points::PointDataGrid::Ptr gridCopy = mGrid->deepCopy();
        runax(mGrid);
        runcpp(gridCopy);
    }

    void printDurations(std::ostream& out) const
    {
        out << "Execution times for case [" << mCaseName << "]" << std::endl;
        out << "CPP duration: " << mCppDuration << std::endl;
        out << "AX duration: " << mAXDuration << std::endl;
    }

    openvdb::ax::Compiler& compiler() { return *mCompiler; }

private:

    inline void runcpp(openvdb::points::PointDataGrid::Ptr g)
    {
        openvdb::util::CpuTimer timer; timer.start();
        mCppFun(g);
        mCppDuration = timer.seconds();
    }

    inline void runax(openvdb::points::PointDataGrid::Ptr g)
    {
        openvdb::ax::PointExecutable::Ptr pointExecutable =
            mCompiler->compile<openvdb::ax::PointExecutable>(mCode);

        openvdb::util::CpuTimer timer; timer.start();
        pointExecutable->execute(*g);
        mAXDuration = timer.seconds();
    }

private:
    std::string mCaseName;
    std::string mCode;
    openvdb::points::PointDataGrid::Ptr mGrid;
    std::function<void(openvdb::points::PointDataGrid::Ptr)> mCppFun;
    openvdb::ax::Compiler::Ptr mCompiler;
    double mCppDuration, mAXDuration;
};

}

}

#endif // OPENVDB_AX_CMD_PERFORMANCECOMPARE_HAS_BEEN_INCLUDED

// Copyright (c) 2015-2019 DNEG
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
