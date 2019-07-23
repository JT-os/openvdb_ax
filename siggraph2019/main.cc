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

#include "PerformanceCompare.h"

#include "CurlNoiseExample.h"
#include "EmptyExample.h"
#include "DragExample.h"
#include "ExtendingAX.h"

#include <openvdb/Grid.h>
#include <openvdb/openvdb.h>
#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointConversion.h>
#include <openvdb/points/PointGroup.h>

#include <iostream>

int main(int argc, char** argv)
{
    using CPPFunction = std::function<void(openvdb::points::PointDataGrid::Ptr)>;
    using TestPair = std::pair<const char*, CPPFunction>;

    // Examples
    const std::map<std::string, TestPair> examples {
        { "drag",   { DragAX,  applyDrag  } },  // Run the drag example through AX and c++
        { "empty",  { EmptyAX, applyEmpty } },  // Run an empty example through AX and c++
        { "curlnoise",  { CurlNoiseAX, applyCurlNoise } },  // Run an empty example through AX and c++
        { "extend", { Mat4InverseAX, applyAXExtension } } // Extend AX with some functions and test them
    };

    // set active test
    const std::string activeTest = "curlnoise";

    if (argc < 3) {
        std::cerr << "ERROR: Insuffucient arguments" << std::endl
            << "Usage: sg2019 <VDB file> <grid name>" << std::endl;
        return 1;
    }

    const std::string vdbFileName(argv[1]);
    const std::string gridName(argv[2]);

    openvdb::initialize();
    openvdb::ax::initialize();

    std::cerr << "Opening File..." << std::endl;
    openvdb::io::File vdbFile(vdbFileName);
    vdbFile.open(/*dely-load*/false);

    std::cerr << "Reading VDB..." << std::endl;
    openvdb::GridBase::Ptr grid = vdbFile.readGrid(gridName);

    openvdb::points::PointDataGrid::ConstPtr pointGrid =
        openvdb::DynamicPtrCast<openvdb::points::PointDataGrid>(grid);

    // Make sure all attributes are loaded
    openvdb::points::prefetch(pointGrid->tree(), /*position*/true, /*attributes*/true);

    vdbFile.close();

    auto test = examples.find(activeTest);
    if (test == examples.end()) {
        std::cerr << "Unable to find test " <<  activeTest << std::endl;
        return 1;
    }

    openvdb::ComparisonCase profiler(activeTest,
        pointGrid->deepCopy(),
        std::string(test->second.first),
        test->second.second);


    if (activeTest == "extend") {
        extendAXFunctions(profiler.compiler());
    }

    std::cerr << "Running Test..." << std::endl;
    std::cerr << test->second.first << std::endl;

    profiler.execute();
    profiler.printDurations(std::cerr);

    openvdb::ax::uninitialize();
    openvdb::uninitialize();

    return 0;
}

// Copyright (c) 2015-2019 DNEG
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
