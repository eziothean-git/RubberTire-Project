using Modding.Mapper;

public sealed class RubberTireConfigMapper : MCustom<string>
{
    public RubberTireConfigMapper(string displayName, string key, string defaultValue)
        : base(displayName, key, defaultValue)
    {
    }

    public override XData SerializeValue(string value)
    {
        return new XString(SerializationKey, value ?? string.Empty);
    }

    public override string DeSerializeValue(XData data)
    {
        XString value = data as XString;
        return value != null ? value.Value : string.Empty;
    }
}

public sealed class RubberTireConfigSelector : CustomSelector<string, RubberTireConfigMapper>
{
    protected override void CreateInterface() { }
    protected override void UpdateInterface() { }
}
