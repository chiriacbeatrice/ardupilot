/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/uavcan/dsdl/uavcan/protocol/DataTypeKind.uavcan
 */

#ifndef UAVCAN_PROTOCOL_DATATYPEKIND_HPP_INCLUDED
#define UAVCAN_PROTOCOL_DATATYPEKIND_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Data type kind (message or service).
#

uint8 SERVICE = 0
uint8 MESSAGE = 1
uint8 value
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.protocol.DataTypeKind
saturated uint8 value
******************************************************************************/

#undef value
#undef SERVICE
#undef MESSAGE

namespace uavcan
{
namespace protocol
{

template <int _tmpl>
struct UAVCAN_EXPORT DataTypeKind_
{
    typedef const DataTypeKind_<_tmpl>& ParameterType;
    typedef DataTypeKind_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > SERVICE;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > MESSAGE;
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > value;
    };

    enum
    {
        MinBitLen
            = FieldTypes::value::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::value::MaxBitLen
    };

    // Constants
    static const typename ::uavcan::StorageType< typename ConstantTypes::SERVICE >::Type SERVICE; // 0
    static const typename ::uavcan::StorageType< typename ConstantTypes::MESSAGE >::Type MESSAGE; // 1

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::value >::Type value;

    DataTypeKind_()
        : value()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<8 == MaxBitLen>::check();
#endif
    }

    bool operator==(ParameterType rhs) const;
    bool operator!=(ParameterType rhs) const { return !operator==(rhs); }

    /**
     * This comparison is based on @ref uavcan::areClose(), which ensures proper comparison of
     * floating point fields at any depth.
     */
    bool isClose(ParameterType rhs) const;

    static int encode(ParameterType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    static int decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
                      ::uavcan::TailArrayOptimizationMode tao_mode = ::uavcan::TailArrayOptEnabled);

    /*
     * Static type info
     */
    enum { DataTypeKind = ::uavcan::DataTypeKindMessage };
    // This type has no default data type ID

    static const char* getDataTypeFullName()
    {
        return "uavcan.protocol.DataTypeKind";
    }

    static void extendDataTypeSignature(::uavcan::DataTypeSignature& signature)
    {
        signature.extend(getDataTypeSignature());
    }

    static ::uavcan::DataTypeSignature getDataTypeSignature();

};

/*
 * Out of line struct method definitions
 */

template <int _tmpl>
bool DataTypeKind_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        value == rhs.value;
}

template <int _tmpl>
bool DataTypeKind_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(value, rhs.value);
}

template <int _tmpl>
int DataTypeKind_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::value::encode(self.value, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int DataTypeKind_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    int res = 1;
    res = FieldTypes::value::decode(self.value, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature DataTypeKind_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x9420A73E008E5930ULL);

    FieldTypes::value::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

template <int _tmpl>
const typename ::uavcan::StorageType< typename DataTypeKind_<_tmpl>::ConstantTypes::SERVICE >::Type
    DataTypeKind_<_tmpl>::SERVICE = 0U; // 0

template <int _tmpl>
const typename ::uavcan::StorageType< typename DataTypeKind_<_tmpl>::ConstantTypes::MESSAGE >::Type
    DataTypeKind_<_tmpl>::MESSAGE = 1U; // 1

/*
 * Final typedef
 */
typedef DataTypeKind_<0> DataTypeKind;

// No default registration

} // Namespace protocol
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::protocol::DataTypeKind >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::protocol::DataTypeKind::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::protocol::DataTypeKind >::stream(Stream& s, ::uavcan::protocol::DataTypeKind::ParameterType obj, const int level)
{
    (void)s;
    (void)obj;
    (void)level;
    if (level > 0)
    {
        s << '\n';
        for (int pos = 0; pos < level; pos++)
        {
            s << "  ";
        }
    }
    s << "value: ";
    YamlStreamer< ::uavcan::protocol::DataTypeKind::FieldTypes::value >::stream(s, obj.value, level + 1);
}

}

namespace uavcan
{
namespace protocol
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::protocol::DataTypeKind::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::protocol::DataTypeKind >::stream(s, obj, 0);
    return s;
}

} // Namespace protocol
} // Namespace uavcan

#endif // UAVCAN_PROTOCOL_DATATYPEKIND_HPP_INCLUDED