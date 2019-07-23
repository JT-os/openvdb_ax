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
#include <openvdb_ax/codegen/Functions.h>
#include <openvdb_ax/codegen/FunctionRegistry.h>
#include <openvdb_ax/compiler/Compiler.h>

const char* Mat4InverseAX = R"AX(

    bool iszero = true;
    for (int i = 0; i < 16; ++i) {
        iszero &= (mat4d@my_mat[i] == 0.0);
    }

    if (iszero) {
        mat4d@my_mat = identity4();
    }

    // inverse a matrix attribute
    mat4d@my_mat = inverse(mat4d@my_mat);

    // smooth step each component
    for (int i = 0; i < 16; ++i) {
        mat4d@my_mat[i] = smoothunitstep(mat4d@my_mat[i]);
    }

)AX";


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


inline double lerp(const double x, const double q0, const double q1)
{
    const double ret = (q0) + x * (q1 - q0);
    return ret;
}

//simple linear inteprolation
struct AXLerp : public openvdb::ax::codegen::FunctionBase
{
    inline const std::string identifier() const override final { return std::string("lerp"); }

    inline static Ptr create(const openvdb::ax::FunctionOptions&) {
        return Ptr(new AXLerp());
    }

    AXLerp() : openvdb::ax::codegen::FunctionBase({
        DECLARE_FUNCTION_SIGNATURE(lerp)
    }) {}
};

// Return 0 if x < 0, 1 if x > 1 or else (3 − 2 x) x².
struct AXSmoothUnitStep : public openvdb::ax::codegen::FunctionBase
{
    inline const std::string identifier() const override final { return std::string("smoothunitstep"); }

    inline static Ptr create(const openvdb::ax::FunctionOptions&) {
        return Ptr(new AXSmoothUnitStep());
    }

    AXSmoothUnitStep() : openvdb::ax::codegen::FunctionBase({
        openvdb::ax::codegen::FunctionSignature<double(double)>
            ::create(static_cast<double(*)(double)>(openvdb::math::SmoothUnitStep)
                , std::string("smoothunitstepd"), 0),
    }) {}
};

// Inverse a 4x4 matrix
struct MatrixInverse : public openvdb::ax::codegen::FunctionBase
{
    struct Internal : public openvdb::ax::codegen::FunctionBase
    {
        inline const std::string identifier() const override final { return std::string("internal_inverse"); }

        inline static Ptr create(const openvdb::ax::FunctionOptions&) { return Ptr(new Internal()); }

        Internal() : openvdb::ax::codegen::FunctionBase({
            DECLARE_FUNCTION_SIGNATURE_OUTPUT(inverse<double>),
            DECLARE_FUNCTION_SIGNATURE_OUTPUT(inverse<float>)
        }) {}

        template <typename T>
        inline static void inverse(T (*in)[16], T (*out)[16])
        {
            openvdb::math::Mat4<T> m1(in[0]);
            m1 = m1.inverse();
            for (size_t i = 0; i < 16; ++i) {
                (*out)[i] = m1.asPointer()[i];
            }
        }
    };

    inline const std::string identifier() const override final { return std::string("inverse"); }

    inline static Ptr create(const openvdb::ax::FunctionOptions&) {
        return Ptr(new MatrixInverse());
    }

    MatrixInverse() : openvdb::ax::codegen::FunctionBase({
        openvdb::ax::codegen::FunctionSignature<openvdb::ax::codegen::M4D*(openvdb::ax::codegen::M4D*)>
            ::create(nullptr, std::string("inversem4d"), 0),
        openvdb::ax::codegen::FunctionSignature<openvdb::ax::codegen::M4F*(openvdb::ax::codegen::M4F*)>
            ::create(nullptr, std::string("inversem4f"), 0)
    }) {}

    inline void
    getDependencies(std::vector<std::string>& identifiers) const override {
        identifiers.emplace_back("internal_inverse");
    }

    llvm::Value*
    generate(const std::vector<llvm::Value*>& args,
         const std::unordered_map<std::string, llvm::Value*>& globals,
         llvm::IRBuilder<>& B) const override final
    {
        std::vector<llvm::Value*> results;
        Internal func;
        func.execute(args, globals, B, &results);
        assert(!results.empty());
        return results.front();
    }
};


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


void extendAXFunctions(openvdb::ax::Compiler& compiler)
{
    std::cerr << "Adding extra functions to AX..." << std::endl;
    openvdb::ax::codegen::FunctionRegistry::UniquePtr reg =
        openvdb::ax::codegen::createDefaultRegistry();
    reg->insert("lerp", AXLerp::create);
    reg->insert("inverse", MatrixInverse::create);
    reg->insert("smoothunitstep", AXSmoothUnitStep::create);
    reg->insert("internal_inverse", MatrixInverse::Internal::create, true);
    compiler.setFunctionRegistry(std::move(reg));
}

void applyAXExtension(openvdb::points::PointDataGrid::Ptr grid)
{
    if (!grid) return;
    auto leaf = grid->tree().cbeginLeaf();
    if (!leaf) return;

    if (!leaf->hasAttribute("my_mat")) {
        openvdb::points::appendAttribute<openvdb::math::Mat4<double>>(grid->tree(), "my_mat");
    }

    openvdb::points::AttributeSet::DescriptorPtr desc = leaf->attributeSet().descriptorPtr();
    const size_t pos = desc->find("my_mat");

    auto matInverseOp = [pos]
        (openvdb::points::PointDataTree::LeafNodeType& leaf, size_t)
    {
        openvdb::points::AttributeWriteHandle<openvdb::math::Mat4<double>>::Ptr matHandle =
            openvdb::points::AttributeWriteHandle<openvdb::math::Mat4<double>>::create(leaf.attributeArray(pos));

        const openvdb::Index count = leaf.getLastValue();
        for (openvdb::Index i = 0; i < count; ++i) {
            openvdb::math::Mat4<double> mat = matHandle->get(i);
            double* ptr = mat.asPointer();

            bool iszero = true;
            for (int i = 0; i < 16; ++i) {
                iszero &= (ptr[i] == 0.0);
            }

            if (iszero) {
                mat = openvdb::math::Mat4<double>::identity();
                ptr = mat.asPointer();
            }

            mat = mat.inverse();
            for (int i = 0; i < 16; ++i) {
                ptr[i] = openvdb::math::SmoothUnitStep<double>(ptr[i]);
            }

            matHandle->set(i, mat);
        }
    };

    openvdb::tree::LeafManager<openvdb::points::PointDataTree> leafManager(grid->tree());
    leafManager.foreach(matInverseOp);
}
