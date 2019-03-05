/*
 * UAVCAN data structure definition for libuavcan.
 *
 * Autogenerated, do not edit.
 *
 * Source file: /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/uavcan/dsdl/uavcan/equipment/ice/1129.FuelTankStatus.uavcan
 */

#ifndef UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_HPP_INCLUDED
#define UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_HPP_INCLUDED

#include <uavcan/build_config.hpp>
#include <uavcan/node/global_data_type_registry.hpp>
#include <uavcan/marshal/types.hpp>

/******************************* Source text **********************************
#
# Generic fuel tank status message.
# All fields are required unless stated otherwise. Unpopulated optional fields should be set to NaN.
#

#
# Reserved for future use.
#
void9

#
# The estimated amount of fuel.
# The reported values can be either measured directly using appropriate sensors,
# or they can be estimated by fusing the data provided by various sensors.
# For example, a Kalman filter can be used to fuse the data from fuel level sensors and flow sensors.
# All fields are required.
#
uint7 available_fuel_volume_percent     # Unit: percent, from 0% to 100%
float32 available_fuel_volume_cm3       # Unit: centimeter^3

#
# Estimate of the current fuel consumption rate.
# The flow can be negative if the fuel is being transferred between the tanks or during refueling.
# This field is required.
# Unit: (centimeter^3)/minute
#
float32 fuel_consumption_rate_cm3pm

#
# Fuel temperature.
# This field is optional, set to NaN if not provided.
# Unit: kelvin
#
float16 fuel_temperature

#
# The ID of the current fuel tank.
#
uint8 fuel_tank_id
******************************************************************************/

/********************* DSDL signature source definition ***********************
uavcan.equipment.ice.FuelTankStatus
void9
saturated uint7 available_fuel_volume_percent
saturated float32 available_fuel_volume_cm3
saturated float32 fuel_consumption_rate_cm3pm
saturated float16 fuel_temperature
saturated uint8 fuel_tank_id
******************************************************************************/

#undef _void_0
#undef available_fuel_volume_percent
#undef available_fuel_volume_cm3
#undef fuel_consumption_rate_cm3pm
#undef fuel_temperature
#undef fuel_tank_id

namespace uavcan
{
namespace equipment
{
namespace ice
{

template <int _tmpl>
struct UAVCAN_EXPORT FuelTankStatus_
{
    typedef const FuelTankStatus_<_tmpl>& ParameterType;
    typedef FuelTankStatus_<_tmpl>& ReferenceType;

    struct ConstantTypes
    {
    };

    struct FieldTypes
    {
        typedef ::uavcan::IntegerSpec< 9, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > _void_0;
        typedef ::uavcan::IntegerSpec< 7, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > available_fuel_volume_percent;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > available_fuel_volume_cm3;
        typedef ::uavcan::FloatSpec< 32, ::uavcan::CastModeSaturate > fuel_consumption_rate_cm3pm;
        typedef ::uavcan::FloatSpec< 16, ::uavcan::CastModeSaturate > fuel_temperature;
        typedef ::uavcan::IntegerSpec< 8, ::uavcan::SignednessUnsigned, ::uavcan::CastModeSaturate > fuel_tank_id;
    };

    enum
    {
        MinBitLen
            = FieldTypes::_void_0::MinBitLen
            + FieldTypes::available_fuel_volume_percent::MinBitLen
            + FieldTypes::available_fuel_volume_cm3::MinBitLen
            + FieldTypes::fuel_consumption_rate_cm3pm::MinBitLen
            + FieldTypes::fuel_temperature::MinBitLen
            + FieldTypes::fuel_tank_id::MinBitLen
    };

    enum
    {
        MaxBitLen
            = FieldTypes::_void_0::MaxBitLen
            + FieldTypes::available_fuel_volume_percent::MaxBitLen
            + FieldTypes::available_fuel_volume_cm3::MaxBitLen
            + FieldTypes::fuel_consumption_rate_cm3pm::MaxBitLen
            + FieldTypes::fuel_temperature::MaxBitLen
            + FieldTypes::fuel_tank_id::MaxBitLen
    };

    // Constants

    // Fields
    typename ::uavcan::StorageType< typename FieldTypes::available_fuel_volume_percent >::Type available_fuel_volume_percent;
    typename ::uavcan::StorageType< typename FieldTypes::available_fuel_volume_cm3 >::Type available_fuel_volume_cm3;
    typename ::uavcan::StorageType< typename FieldTypes::fuel_consumption_rate_cm3pm >::Type fuel_consumption_rate_cm3pm;
    typename ::uavcan::StorageType< typename FieldTypes::fuel_temperature >::Type fuel_temperature;
    typename ::uavcan::StorageType< typename FieldTypes::fuel_tank_id >::Type fuel_tank_id;

    FuelTankStatus_()
        : available_fuel_volume_percent()
        , available_fuel_volume_cm3()
        , fuel_consumption_rate_cm3pm()
        , fuel_temperature()
        , fuel_tank_id()
    {
        ::uavcan::StaticAssert<_tmpl == 0>::check();  // Usage check

#if UAVCAN_DEBUG
        /*
         * Cross-checking MaxBitLen provided by the DSDL compiler.
         * This check shall never be performed in user code because MaxBitLen value
         * actually depends on the nested types, thus it is not invariant.
         */
        ::uavcan::StaticAssert<104 == MaxBitLen>::check();
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
    enum { DefaultDataTypeID = 1129 };

    static const char* getDataTypeFullName()
    {
        return "uavcan.equipment.ice.FuelTankStatus";
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
bool FuelTankStatus_<_tmpl>::operator==(ParameterType rhs) const
{
    return
        available_fuel_volume_percent == rhs.available_fuel_volume_percent &&
        available_fuel_volume_cm3 == rhs.available_fuel_volume_cm3 &&
        fuel_consumption_rate_cm3pm == rhs.fuel_consumption_rate_cm3pm &&
        fuel_temperature == rhs.fuel_temperature &&
        fuel_tank_id == rhs.fuel_tank_id;
}

template <int _tmpl>
bool FuelTankStatus_<_tmpl>::isClose(ParameterType rhs) const
{
    return
        ::uavcan::areClose(available_fuel_volume_percent, rhs.available_fuel_volume_percent) &&
        ::uavcan::areClose(available_fuel_volume_cm3, rhs.available_fuel_volume_cm3) &&
        ::uavcan::areClose(fuel_consumption_rate_cm3pm, rhs.fuel_consumption_rate_cm3pm) &&
        ::uavcan::areClose(fuel_temperature, rhs.fuel_temperature) &&
        ::uavcan::areClose(fuel_tank_id, rhs.fuel_tank_id);
}

template <int _tmpl>
int FuelTankStatus_<_tmpl>::encode(ParameterType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    typename ::uavcan::StorageType< typename FieldTypes::_void_0 >::Type _void_0 = 0;
    int res = 1;
    res = FieldTypes::_void_0::encode(_void_0, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::available_fuel_volume_percent::encode(self.available_fuel_volume_percent, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::available_fuel_volume_cm3::encode(self.available_fuel_volume_cm3, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::fuel_consumption_rate_cm3pm::encode(self.fuel_consumption_rate_cm3pm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::fuel_temperature::encode(self.fuel_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::fuel_tank_id::encode(self.fuel_tank_id, codec,  tao_mode);
    return res;
}

template <int _tmpl>
int FuelTankStatus_<_tmpl>::decode(ReferenceType self, ::uavcan::ScalarCodec& codec,
    ::uavcan::TailArrayOptimizationMode tao_mode)
{
    (void)self;
    (void)codec;
    (void)tao_mode;
    typename ::uavcan::StorageType< typename FieldTypes::_void_0 >::Type _void_0 = 0;
    int res = 1;
    res = FieldTypes::_void_0::decode(_void_0, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::available_fuel_volume_percent::decode(self.available_fuel_volume_percent, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::available_fuel_volume_cm3::decode(self.available_fuel_volume_cm3, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::fuel_consumption_rate_cm3pm::decode(self.fuel_consumption_rate_cm3pm, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::fuel_temperature::decode(self.fuel_temperature, codec,  ::uavcan::TailArrayOptDisabled);
    if (res <= 0)
    {
        return res;
    }
    res = FieldTypes::fuel_tank_id::decode(self.fuel_tank_id, codec,  tao_mode);
    return res;
}

/*
 * Out of line type method definitions
 */
template <int _tmpl>
::uavcan::DataTypeSignature FuelTankStatus_<_tmpl>::getDataTypeSignature()
{
    ::uavcan::DataTypeSignature signature(0x286B4A387BA84BC4ULL);

    FieldTypes::_void_0::extendDataTypeSignature(signature);
    FieldTypes::available_fuel_volume_percent::extendDataTypeSignature(signature);
    FieldTypes::available_fuel_volume_cm3::extendDataTypeSignature(signature);
    FieldTypes::fuel_consumption_rate_cm3pm::extendDataTypeSignature(signature);
    FieldTypes::fuel_temperature::extendDataTypeSignature(signature);
    FieldTypes::fuel_tank_id::extendDataTypeSignature(signature);

    return signature;
}

/*
 * Out of line constant definitions
 */

/*
 * Final typedef
 */
typedef FuelTankStatus_<0> FuelTankStatus;

namespace
{

const ::uavcan::DefaultDataTypeRegistrator< ::uavcan::equipment::ice::FuelTankStatus > _uavcan_gdtr_registrator_FuelTankStatus;

}

} // Namespace ice
} // Namespace equipment
} // Namespace uavcan

/*
 * YAML streamer specialization
 */
namespace uavcan
{

template <>
class UAVCAN_EXPORT YamlStreamer< ::uavcan::equipment::ice::FuelTankStatus >
{
public:
    template <typename Stream>
    static void stream(Stream& s, ::uavcan::equipment::ice::FuelTankStatus::ParameterType obj, const int level);
};

template <typename Stream>
void YamlStreamer< ::uavcan::equipment::ice::FuelTankStatus >::stream(Stream& s, ::uavcan::equipment::ice::FuelTankStatus::ParameterType obj, const int level)
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
    s << "available_fuel_volume_percent: ";
    YamlStreamer< ::uavcan::equipment::ice::FuelTankStatus::FieldTypes::available_fuel_volume_percent >::stream(s, obj.available_fuel_volume_percent, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "available_fuel_volume_cm3: ";
    YamlStreamer< ::uavcan::equipment::ice::FuelTankStatus::FieldTypes::available_fuel_volume_cm3 >::stream(s, obj.available_fuel_volume_cm3, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "fuel_consumption_rate_cm3pm: ";
    YamlStreamer< ::uavcan::equipment::ice::FuelTankStatus::FieldTypes::fuel_consumption_rate_cm3pm >::stream(s, obj.fuel_consumption_rate_cm3pm, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "fuel_temperature: ";
    YamlStreamer< ::uavcan::equipment::ice::FuelTankStatus::FieldTypes::fuel_temperature >::stream(s, obj.fuel_temperature, level + 1);
    s << '\n';
    for (int pos = 0; pos < level; pos++)
    {
        s << "  ";
    }
    s << "fuel_tank_id: ";
    YamlStreamer< ::uavcan::equipment::ice::FuelTankStatus::FieldTypes::fuel_tank_id >::stream(s, obj.fuel_tank_id, level + 1);
}

}

namespace uavcan
{
namespace equipment
{
namespace ice
{

template <typename Stream>
inline Stream& operator<<(Stream& s, ::uavcan::equipment::ice::FuelTankStatus::ParameterType obj)
{
    ::uavcan::YamlStreamer< ::uavcan::equipment::ice::FuelTankStatus >::stream(s, obj, 0);
    return s;
}

} // Namespace ice
} // Namespace equipment
} // Namespace uavcan

#endif // UAVCAN_EQUIPMENT_ICE_FUELTANKSTATUS_HPP_INCLUDED