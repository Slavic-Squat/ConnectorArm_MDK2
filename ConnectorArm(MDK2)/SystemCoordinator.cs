using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program
    {
        public class SystemCoordinator
        {
            public static double SystemTime { get; private set; }

            public CraneArm CraneArm { get; private set; }
            public UserInput UserInput { get; private set; }

            private IMyShipController _controller;
            
            public SystemCoordinator()
            {
                _controller = GTS.GetBlockWithName("Arm Controller") as IMyShipController;
                CraneArm = new CraneArm(1f, 1f, false, 10f, 10f);
                UserInput = new UserInput(_controller);
            }

            public void Run()
            {
                SystemTime += RuntimeInfo.TimeSinceLastRun.TotalSeconds;
                UserInput.Run(SystemTime);
                CraneArm.Control(UserInput);
            }
        }
    }
}
