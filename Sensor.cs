using System.ComponentModel;
using System.Runtime.CompilerServices;

namespace BENCHLAB_Core
{
    public enum SensorType
    {
        Temperature,
        Humidity,
        Duty,
        Revolutions,
        Voltage,
        Current,
        Power,
        Clock,
        Usage,
        Dummy,
        Other
    }

    public class Sensor
    {
        public Sensor(int id, string shortName, string name, SensorType type)
        {
            ShortName = shortName;
            Name = name;
            Type = type;
        }

        public int Id { get; }
        public string ShortName { get; }
        public string Name { get; }
        public SensorType Type { get; }

        public double Value
        {
            get => _value;
            set
            {
                _value = value;
            }
        }

        bool _is_valid = false;
        public bool IsValid
        {
            get => _is_valid;
            set
            {
                _is_valid = value;
            }
        }

        private double _value;

    }
}
