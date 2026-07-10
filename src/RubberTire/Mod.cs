using System;
using UnityEngine;
using Modding;
using Modding.Mapper;

namespace RubberTire
{
    public class Mod : ModEntryPoint
    {
        public override void OnLoad()
        {
            CustomMapperTypes.AddMapperType<string, global::RubberTireConfigMapper, global::RubberTireConfigSelector>();

            if (!Mods.IsModLoaded(new Guid(global::RubberTireWheelScript.FactoryUiModGuid)))
            {
                Debug.LogWarning(
                    "RubberTire: UIFactory 3 is required for tyre parameter editing.");
                return;
            }

            GameObject controllerRoot = GameObject.Find("ModControllerObject");
            if (controllerRoot == null)
            {
                controllerRoot = new GameObject("ModControllerObject");
                UnityEngine.Object.DontDestroyOnLoad(controllerRoot);
            }

            if (controllerRoot.GetComponent<global::RubberTireFactoryUIController>() == null)
                controllerRoot.AddComponent<global::RubberTireFactoryUIController>();
            if (controllerRoot.GetComponent<global::RubberTireEngineDashboard>() == null)
                controllerRoot.AddComponent<global::RubberTireEngineDashboard>();
        }
    }
}
