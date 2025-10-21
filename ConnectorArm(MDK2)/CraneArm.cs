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
            private Vector3 _seg0Vector;
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

            public bool EEControlled { get; private set; } = false;
            public bool ArmControlled { get; private set; } = true;

            public CraneArm(float sensitivity, float speed, bool cylindricalMode)
            {
                _joint0 = new Rotor("Joint0");
                _joint1 = new Rotor("Joint1");
                _joint2 = new Rotor("Joint2");
                //_eePitchJoint = new Rotor("EE Pitch Joint");
                //_eeYawJoint = new Rotor("EE Yaw Joint");
                //_eeRollJoint = new Rotor("EE Roll Joint");

                _sensitivity = sensitivity;
                _speed = speed;
                _cylindricalMode = cylindricalMode;

                _seg0Vector = new Vector3(0, 0, 0);
                _seg1Vector = new Vector3(0, 0, -10);
                _seg2Vector = new Vector3(0, 0, -10);
            }

            public void Control(UserInput input)
            {
                Matrix H0 = Matrix.CreateRotationY(_joint0.CurrentAngle);
                Matrix H1 = Matrix.CreateRotationX(_joint1.CurrentAngle);
                H1.Translation = _seg0Vector;
                Matrix H2 = Matrix.CreateRotationX(_joint2.CurrentAngle);
                H2.Translation = _seg1Vector;
                Matrix H3 = Matrix.Identity;
                H3.Translation = _seg2Vector;

                Matrix HT = H3 * H2 * H1 * H0;
                Vector3 currentCoord = HT.Translation;

                DebugDraw.DrawMatrix(HT * _joint0.RotorBlock.WorldMatrix, length: 2f);

                Vector3 J0v = Vector3.Cross(H0.Up, currentCoord - H0.Translation);
                Vector3 J0w = H0.Up;
                double[] J0 = new double[] { J0v.X, J0v.Y, J0v.Z, J0w.X, J0w.Y, J0w.Z };

                Vector3 J1v = Vector3.Cross((H1*H0).Right, currentCoord - (H1 * H0).Translation);
                Vector3 J1w = (H1 * H0).Right;
                double[] J1 = new double[] { J1v.X, J1v.Y, J1v.Z, J1w.X, J1w.Y, J1w.Z };

                Vector3 J2v = Vector3.Cross((H2*H1*H0).Right, currentCoord - (H2 * H1 * H0).Translation);
                Vector3 J2w = (H2 * H1 * H0).Right;
                double[] J2 = new double[] { J2v.X, J2v.Y, J2v.Z, J2w.X, J2w.Y, J2w.Z };

                double[,] J = new double[3, 6]
                {
                    { J0[0], J0[1], J0[2], J0[3], J0[4], J0[5] },
                    { J1[0], J1[1], J1[2], J1[3], J1[4], J1[5] },
                    { J2[0], J2[1], J2[2], J2[3], J2[4], J2[5] },
                };

                for (int i = 0; i < 3; i++)
                {
                    DebugEcho("\n");
                    for (int j = 0; j < 6; j++)
                    {
                        DebugEcho(J[i, j].ToString("F3") + " ");
                    }
                }

                double[,] J_pseudoInv = MyMath.DampedPseudoInverse(J, 0.1);
            }
        }
    }
}
