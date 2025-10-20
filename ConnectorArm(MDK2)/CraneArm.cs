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
        public class CraneArm
        {
            private Rotor _joint0;
            private Rotor _joint1;
            private Vector3 _seg1Vector;
            private Rotor _joint2;
            private Vector3 _seg2Vector;
            private Rotor _eePitchJoint;
            private Rotor _eeYawJoint;
            private Rotor _eeRollJoint;

            private float _sensitivity;
            private float _speed;
            private bool _cylindricalMode;
            private bool _OOB = false;

            private Vector3 _targetCoord;
            private Matrix _targetEEO_base;

            public bool EEControlled { get; private set; } = false;
            public bool ArmControlled { get; private set; } = true;

            public CraneArm(float sensitivity, float speed, bool cylindricalMode, float seg1Length, float seg2Length)
            {
                _joint0 = new Rotor("Joint0");
                _joint1 = new Rotor("Joint1");
                _joint2 = new Rotor("Joint2");
                _eePitchJoint = new Rotor("EE Pitch Joint");
                _eeYawJoint = new Rotor("EE Yaw Joint");
                _eeRollJoint = new Rotor("EE Roll Joint");

                _sensitivity = sensitivity;
                _speed = speed;
                _cylindricalMode = cylindricalMode;

                _seg1Vector = new Vector3(0, 0, -seg1Length);
                _seg2Vector = new Vector3(0, 0, -seg2Length);
            }

            public void Control(UserInput input)
            {
                Matrix H0 = Matrix.CreateRotationY(_joint0.CurrentAngle);
                Matrix H1 = Matrix.CreateRotationX(_joint1.CurrentAngle);
                H1.Translation = new Vector3(0, 2.5f, 0);
                Matrix H2 = Matrix.CreateRotationX(_joint2.CurrentAngle);
                H2.Translation = _seg1Vector;
                Matrix H3 = Matrix.Identity;
                H3.Translation = _seg2Vector;

                Matrix HT = H3 * H2 * H1 * H0;
                Vector3 currentCoord = HT.Translation;
            }
        }
    }
}
