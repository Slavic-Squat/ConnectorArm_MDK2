﻿using Sandbox.Game.EntityComponents;
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
                _eePitchJoint = new Rotor("EE Pitch Joint");
                _eeYawJoint = new Rotor("EE Yaw Joint");
                _eeRollJoint = new Rotor("EE Roll Joint");

                _sensitivity = sensitivity;
                _speed = speed;
                _cylindricalMode = cylindricalMode;

                _seg0Vector = new Vector3(2.5f, 2.5f, 0);
                _seg1Vector = new Vector3(-2.5f, 0, -10);
                _seg2Vector = new Vector3(0, 0, -10);
            }

            public void Control(UserInput input)
            {
                Matrix H0 = Matrix.CreateRotationY(_joint0.CurrentAngle);
                Matrix H1 = Matrix.CreateRotationX(_joint1.CurrentAngle);
                H1.Translation = _seg0Vector;
                Matrix H2 = Matrix.CreateRotationX(_joint2.CurrentAngle);
                H2.Translation = _seg1Vector;
                Matrix H3 = Matrix.CreateRotationY(_eeYawJoint.CurrentAngle);
                H3.Translation = _seg2Vector;
                Matrix H4 = Matrix.CreateRotationX(_eePitchJoint.CurrentAngle);
                Matrix H5 = Matrix.CreateRotationZ(_eeRollJoint.CurrentAngle);

                Matrix HT = H5 * H4 * H3 * H2 * H1 * H0;
                Vector3 currentCoord = HT.Translation;

                //DebugDraw.DrawMatrix(HT * _joint0.RotorBlock.WorldMatrix, length: 2f);

                Vector3 J0v = Vector3.Cross(H0.Up, currentCoord - H0.Translation);
                Vector3 J0w = H0.Up;
                double[] J0 = new double[6] { J0v.X, J0v.Y, J0v.Z, J0w.X, J0w.Y, J0w.Z };

                Matrix H0_1 = H1 * H0;
                Vector3 J1v = Vector3.Cross(H0_1.Right, currentCoord - H0_1.Translation);
                Vector3 J1w = H0_1.Right;
                double[] J1 = new double[6] { J1v.X, J1v.Y, J1v.Z, J1w.X, J1w.Y, J1w.Z };

                Matrix H0_2 = H2 * H1 * H0;
                Vector3 J2v = Vector3.Cross(H0_2.Right, currentCoord - H0_2.Translation);
                Vector3 J2w = H0_2.Right;
                double[] J2 = new double[6] { J2v.X, J2v.Y, J2v.Z, J2w.X, J2w.Y, J2w.Z };

                Matrix H0_3 = H3 * H2 * H1 * H0;
                Vector3 J3v = Vector3.Cross(H0_3.Up, currentCoord - H0_3.Translation);
                Vector3 J3w = H0_3.Up;
                double[] J3 = new double[6] { J3v.X, J3v.Y, J3v.Z, J3w.X, J3w.Y, J3w.Z };

                Matrix H0_4 = H4 * H3 * H2 * H1 * H0;
                Vector3 J4v = Vector3.Cross(H0_4.Right, currentCoord - H0_4.Translation);
                Vector3 J4w = H0_4.Right;
                double[] J4 = new double[6] { J4v.X, J4v.Y, J4v.Z, J4w.X, J4w.Y, J4w.Z };

                Matrix H0_5 = H5 * H4 * H3 * H2 * H1 * H0;
                Vector3 J5v = Vector3.Cross(H0_5.Backward, currentCoord - H0_5.Translation);
                Vector3 J5w = H0_5.Backward;
                double[] J5 = new double[6] { J5v.X, J5v.Y, J5v.Z, J5w.X, J5w.Y, J5w.Z };

                double[,] J = new double[6, 6]
                {
                    { J0[0], J0[1], J0[2], J0[3], J0[4], J0[5] },
                    { J1[0], J1[1], J1[2], J1[3], J1[4], J1[5] },
                    { J2[0], J2[1], J2[2], J2[3], J2[4], J2[5] },
                    { J3[0], J3[1], J3[2], J3[3], J3[4], J3[5] },
                    { J4[0], J4[1], J4[2], J4[3], J4[4], J4[5] },
                    { J5[0], J5[1], J5[2], J5[3], J5[4], J5[5] }
                };

                double[,] J_pseudoInv = MyMath.DampedPseudoInverse(J, 0.1);

                double[] inputSignal = new double[6];

                if (ArmControlled)
                {
                    Vector3 trans0 = Vector3.Zero;
                    Vector3 trans1 = Vector3.Zero;
                    Vector3 trans2 = Vector3.Zero;

                    if (input.WPress) trans0 = -1f * H0_5.Backward;
                    if (input.SPress) trans0 = 1f * H0_5.Backward;
                    if (input.APress) trans1 = -1f * H0_5.Right;
                    if (input.DPress) trans1 = 1f * H0_5.Right;
                    if (input.SpacePress) trans2 = 1f * H0_5.Up;
                    if (input.CPress) trans2 = -1f * H0_5.Up;

                    Vector3 transInput = trans0 + trans1 + trans2;

                    inputSignal[0] = transInput.X;
                    inputSignal[1] = transInput.Y;
                    inputSignal[2] = transInput.Z;
                }

                if (EEControlled)
                {
                    Vector3 rot0 = Vector3.Zero;
                    Vector3 rot1 = Vector3.Zero;
                    Vector3 rot2 = Vector3.Zero;

                    if (input.WPress) rot1 = 1f * H0_5.Right;
                    if (input.SPress) rot1 = -1f * H0_5.Right;
                    if (input.APress) rot0 = 1f * H0_5.Up;
                    if (input.DPress) rot0 = -1f * H0_5.Up;
                    if (input.QPress) rot2 = 1f * H0_5.Backward;
                    if (input.EPress) rot2 = -1f * H0_5.Backward;

                    Vector3 rotInput = rot0 + rot1 + rot2;

                    inputSignal[3] = rotInput.X;
                    inputSignal[4] = rotInput.Y;
                    inputSignal[5] = rotInput.Z;
                }
                
                double[] outputSignal = MyMath.MultiplyVectorMatrix(inputSignal, J_pseudoInv);

                _joint0.Velocity = (float)outputSignal[0];
                _joint1.Velocity = (float)outputSignal[1];
                _joint2.Velocity = (float)outputSignal[2];
                _eeYawJoint.Velocity = (float)outputSignal[3];
                _eePitchJoint.Velocity = (float)outputSignal[4];
                _eeRollJoint.Velocity = (float)outputSignal[5];

                if (_joint0.IsMaxedOut || _joint1.IsMaxedOut || _joint2.IsMaxedOut || _eeYawJoint.IsMaxedOut || _eePitchJoint.IsMaxedOut || _eeRollJoint.IsMaxedOut)
                {
                    _joint0.Velocity = 0;
                    _joint1.Velocity = 0;
                    _joint2.Velocity = 0;
                    _eeYawJoint.Velocity = 0;
                    _eePitchJoint.Velocity = 0;
                    _eeRollJoint.Velocity = 0;
                }
            }

            public bool ToggleControlMode()
            {
                ArmControlled = !ArmControlled;
                EEControlled = !EEControlled;
                return true;
            }
        }
    }
}
