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

#include <openvdb/openvdb.h>
#include <openvdb/points/PointAttribute.h>
#include <openvdb/points/PointDataGrid.h>
#include <openvdb/points/PointGroup.h>

const char* DragAX = R"AX(

if (!ingroup("ballistic")) return;

float dt = 1.0f / (4.0f * 24.0f);      // timestep
vec3f gravity = {0.0f, -9.81f, 0.0f}; // gravity
vec3f dV = {2.0f, 0.0f, 0.0f} - v@v;  // drag
float lengthV = length(dV);

// clamp valid radii
float@rad = clamp(float@rad, 0.0001f, 1.0f);

float Re = lengthV * float@rad / 1.225f;
float C = 0.0f;
if (Re > 1000.0f) C = 24.0f / Re;
else              C = 0.424f;

// calculate drag force
vec3f drag = 0.5f * 1.2f * C * lengthV * dV * 4.0f
    * 3.14f * pow(float@rad, 2.0f);

// update velocity
vec3f@v += (gravity -  drag / ((4.0f / 3.0f)
    * 3.14f * pow(float@rad, 3.0f))) * dt;

)AX";


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


void applyDrag(openvdb::points::PointDataGrid::Ptr grid)
{
    if (!grid) return;
    auto leaf = grid->tree().cbeginLeaf();
    if (!leaf) return;

    if (!leaf->hasAttribute("rad")) {
        openvdb::points::appendAttribute<float>(grid->tree(), "rad");
    }
    if (!leaf->hasAttribute("v")) {
        openvdb::points::appendAttribute<openvdb::Vec3f>(grid->tree(), "v");
    }
    openvdb::points::AttributeSet::DescriptorPtr desc = leaf->attributeSet().descriptorPtr();

    openvdb::points::AttributeSet::Descriptor::GroupIndex groupIndex;

    bool hasGroup = false;
    if (desc->hasGroup("ballistic")) {
        groupIndex = leaf->attributeSet().groupIndex("ballistic");
        hasGroup = true;
    }

    const size_t rpos = desc->find("rad");
    const size_t vpos = desc->find("v");

    auto dragOp = [rpos,vpos,hasGroup,&groupIndex]
        (openvdb::points::PointDataTree::LeafNodeType& leaf, size_t)
    {
        // Attribute handles
        openvdb::points::AttributeWriteHandle<openvdb::Vec3f>::Ptr vHandle =
            openvdb::points::AttributeWriteHandle<openvdb::Vec3f>::create(leaf.attributeArray(vpos));
        openvdb::points::AttributeWriteHandle<float>::Ptr rHandle =
            openvdb::points::AttributeWriteHandle<float>::create(leaf.attributeArray(rpos));

        // Group Filter
        openvdb::points::GroupFilter filter(groupIndex);
        filter.reset(leaf);

        //for (auto iter = leaf.beginIndexAll(); iter; ++iter)

        const auto count = leaf.getLastValue();
        for (auto iter = 0; iter < count; ++iter)
        {
            if (hasGroup && !filter.valid(&iter)) continue;

            const float dt = 1.0f / (4.0f * 24.0f);     // timestep
            const openvdb::Vec3f gravity = {0.0f, -9.81f, 0.0f}; // gravity
            const openvdb::Vec3f cdV = {2.0f, 0.0f, 0.0f};  // drag

            openvdb::Vec3f vel = vHandle->get(iter);
            float rad = rHandle->get(iter);

            const openvdb::Vec3f dV = cdV - vel;
            const float lengthV = dV.length();
            rad = openvdb::math::Clamp(rad, 0.0001f, 1.0f);

            const float Re = lengthV * rad / 1.225f;
            float C = 0.0f;
            if (Re > 1000.0f) C = 24.0f / Re;
            else              C = 0.424f;

            // calculate drag force
            // using powf to match pow selection in aX when given floats
            const openvdb::Vec3f drag = 0.5f * 1.2f * C
                * lengthV * dV * 4.0f * 3.14f * powf(rad, 2.0f);

            // update velocity
            vel += (gravity - drag / ((4.0f / 3.0f) * 3.14f * powf(rad, 3.0f))) * dt;

            // set values
            vHandle->set(iter, vel);
            rHandle->set(iter, rad);
        }
    };

    openvdb::tree::LeafManager<openvdb::points::PointDataTree> leafManager(grid->tree());
    leafManager.foreach(dragOp);
}
