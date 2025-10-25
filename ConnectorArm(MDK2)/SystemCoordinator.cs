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

            public MyIni Config { get; private set; }
            public CommandHandler CommandHandler { get; private set; }
            public CraneArm CraneArm { get; private set; }
            public UserInput UserInput { get; private set; }

            private IMyShipController _controller;
            private IMyTerminalBlock _storageBlock;

            private IMyLightingBlock _eeStatusLight;
            private IMyLightingBlock _armStatusLight;

            private Dictionary<string, Action<string[]>> _commands = new Dictionary<string, Action<string[]>>();

            public SystemCoordinator()
            {
                GetBlocks();
                Init();                
            }

            private void GetBlocks()
            {
                _controller = GTS.GetBlockWithName("Arm Controller") as IMyShipController;
                if (_controller == null)
                    throw new Exception("Arm Controller not found");
                _storageBlock = _controller;
            }

            private void Init()
            {
                CraneArm = new CraneArm(1f, 1f, false);
                UserInput = new UserInput(_controller);
                CommandHandler = new CommandHandler(MePB, _commands);

                _commands["TOGGLE_CTRL_MODE"] = (args) => ToggleControlMode();
            }

            public void Run()
            {
                SystemTime += RuntimeInfo.TimeSinceLastRun.TotalSeconds;
                UserInput.Run(SystemTime);
                CraneArm.Control(UserInput);
            }

            public bool Command(string command)
            {
                return CommandHandler.RunCommands(command);
            }

            private bool ToggleControlMode()
            {
                return CraneArm.ToggleControlMode();
            }
        }
    }
}
