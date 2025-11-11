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
        public static class MyMath
        {
            public static double[,] InverseGaussJordan(double[,] matrix)
            {
                int n = matrix.GetLength(0);
                if (n != matrix.GetLength(1))
                    throw new ArgumentException("Matrix must be square");

                // Create augmented matrix [A | I]
                double[,] aug = new double[n, 2 * n];
                for (int i = 0; i < n; i++)
                {
                    for (int j = 0; j < n; j++)
                        aug[i, j] = matrix[i, j];
                    aug[i, n + i] = 1.0; // Identity on right side
                }

                // Forward elimination with partial pivoting
                for (int col = 0; col < n; col++)
                {
                    // Find pivot
                    int pivotRow = col;
                    double maxVal = Math.Abs(aug[col, col]);
                    for (int i = col + 1; i < n; i++)
                    {
                        if (Math.Abs(aug[i, col]) > maxVal)
                        {
                            maxVal = Math.Abs(aug[i, col]);
                            pivotRow = i;
                        }
                    }

                    // Check for singularity
                    if (Math.Abs(maxVal) < 1e-10)
                        throw new InvalidOperationException("Matrix is singular or near-singular");

                    // Swap rows
                    if (pivotRow != col)
                    {
                        for (int j = 0; j < 2 * n; j++)
                        {
                            double temp = aug[col, j];
                            aug[col, j] = aug[pivotRow, j];
                            aug[pivotRow, j] = temp;
                        }
                    }

                    // Scale pivot row
                    double pivot = aug[col, col];
                    for (int j = 0; j < 2 * n; j++)
                        aug[col, j] /= pivot;

                    // Eliminate column
                    for (int i = 0; i < n; i++)
                    {
                        if (i != col)
                        {
                            double factor = aug[i, col];
                            for (int j = 0; j < 2 * n; j++)
                                aug[i, j] -= factor * aug[col, j];
                        }
                    }
                }

                // Extract inverse from right half
                double[,] inverse = new double[n, n];
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++)
                        inverse[i, j] = aug[i, n + j];

                return inverse;
            }

            // ============================================================================
            // METHOD 2: DAMPED LEAST SQUARES - Most Practical for Robotics
            // ============================================================================
            // Formula: J† = J^T(JJ^T + λ²I)^(-1)
            // Use when: Any size, especially near singularities

            public static double[,] DampedPseudoInverseWide(double[,] J, double lambda = 0.01)
            {
                int m = J.GetLength(0); // rows
                int n = J.GetLength(1); // cols

                // Compute JJ^T
                double[,] JJT = MultiplyMatrices(J, Transpose(J));

                // Add damping: JJ^T + λ²I
                for (int i = 0; i < m; i++)
                    JJT[i, i] += lambda * lambda;

                // Invert (JJ^T + λ²I)
                double[,] JJT_inv = InverseGaussJordan(JJT);

                // Compute J^T * (JJ^T + λ²I)^(-1)
                return MultiplyMatrices(Transpose(J), JJT_inv);
            }

            public static double[,] DampedPseudoInverseTall(double[,] J, double lambda = 0.01)
            {
                int m = J.GetLength(0); // rows
                int n = J.GetLength(1); // cols

                // Compute J^T*J
                double[,] JTJ = MultiplyMatrices(Transpose(J), J);

                // Add damping: J^T*J + λ²I
                for (int i = 0; i < n; i++)
                    JTJ[i, i] += lambda * lambda;

                // Invert (J^T*J + λ²I)
                double[,] JTJ_inv = InverseGaussJordan(JTJ);

                // Compute (J^T*J + λ²I)^(-1)*J^T
                return MultiplyMatrices(JTJ_inv, Transpose(J));
            }

            public static double[,] DampedWeightedPseudoInverseTall(double[,] J, double[] taskWeights = null, double[] jointWeights = null, double lambda = 0.01)
            {
                int m = J.GetLength(0); // rows
                int n = J.GetLength(1); // cols

                // Create weight matrices
                double[,] Wt = new double[m, m];

                if (taskWeights == null)
                {
                    taskWeights = new double[m];
                    for (int i = 0; i < m; i++)
                        taskWeights[i] = 1.0;
                }
                if (jointWeights == null)
                {
                    jointWeights = new double[n];
                    for (int i = 0; i < n; i++)
                        jointWeights[i] = 1.0;
                }

                for (int i = 0; i < m; i++)
                    Wt[i, i] = taskWeights[i];

                // Compute J^T * Wt * J
                double[,] JT_Wt_J = MultiplyMatrices(MultiplyMatrices(Transpose(J), Wt), J);

                // Add damping: J^T * Wt * J + λ² * Wj
                for (int i = 0; i < n; i++)
                    JT_Wt_J[i, i] += lambda * lambda * jointWeights[i];

                // Invert
                double[,] invTerm = InverseGaussJordan(JT_Wt_J);

                // Compute weighted pseudo-inverse: invTerm * J^T * Wt
                return MultiplyMatrices(MultiplyMatrices(invTerm, Transpose(J)), Wt);
            }

            public static double[,] DampedWeightedPseudoInverseWide(double[,] J, double[] taskWeights = null, double[] jointWeights = null, double lambda = 0.01)
            {
                int m = J.GetLength(0); // rows
                int n = J.GetLength(1); // cols

                // Create weight matrices
                double[,] Wj_inv = new double[n, n];

                if (taskWeights == null)
                {
                    taskWeights = new double[m];
                    for (int i = 0; i < m; i++)
                        taskWeights[i] = 1.0;
                }
                if (jointWeights == null)
                {
                    jointWeights = new double[n];
                    for (int i = 0; i < n; i++)
                        jointWeights[i] = 1.0;
                }
                for (int j = 0; j < n; j++)
                    Wj_inv[j, j] = 1 / jointWeights[j];

                // Compute J * Wj_inv * J^T
                double[,] J_Wji_JT = MultiplyMatrices(MultiplyMatrices(J, Wj_inv), Transpose(J));

                // Add damping: J * Wj_inv * J^T + λ² * Wt_inv
                for (int i = 0; i < m; i++)
                    J_Wji_JT[i, i] += lambda * lambda * 1 / taskWeights[i];

                // Invert
                double[,] invTerm = InverseGaussJordan(J_Wji_JT);

                // Compute weighted pseudo-inverse: Wj_inv * J^T * invTerm
                return MultiplyMatrices(MultiplyMatrices(Wj_inv, Transpose(J)), invTerm);
            }

            public static double[,] NullSpaceProjector(double[,] J, double[,] J_pi)
            {
                // N = I - J†J
                int n = J.GetLength(1); // cols
                double[,] I = new double[n, n];
                for (int i = 0; i < n; i++)
                    I[i, i] = 1.0;
                double[,] J_pi_J = MultiplyMatrices(J_pi, J);
                return AddMatrices(I, MultiplyScalar(J_pi_J, -1.0));
            }

            // ============================================================================
            // METHOD 3: QR DECOMPOSITION PSEUDO-INVERSE - Better Numerical Stability
            // ============================================================================
            // More complex but more stable than Gauss-Jordan

            public static double[,] PseudoInverseQR(double[,] A)
            {
                int m = A.GetLength(0);
                int n = A.GetLength(1);

                // QR decomposition: A = QR
                var qrTuple = QRDecomposition(A);
                double[,] Q = qrTuple.Item1;
                double[,] R = qrTuple.Item2;

                // For overdetermined (m >= n): A† = R^(-1) Q^T
                // For underdetermined (m < n): A† = Q R^(-1)

                if (m >= n)
                {
                    // Take first n rows of R (it's upper triangular)
                    double[,] R_square = new double[n, n];
                    for (int i = 0; i < n; i++)
                        for (int j = 0; j < n; j++)
                            R_square[i, j] = R[i, j];

                    double[,] R_inv = InverseUpperTriangular(R_square);
                    return MultiplyMatrices(R_inv, Transpose(Q));
                }
                else
                {
                    // Transpose problem and solve
                    throw new Exception("Underdetermined case - use damped LS instead");
                }
            }

            // ============================================================================
            // HELPER: QR Decomposition (Gram-Schmidt)
            // ============================================================================

            public static MyTuple<double[,], double[,]> QRDecomposition(double[,] A)
            {
                int m = A.GetLength(0);
                int n = A.GetLength(1);

                double[,] Q = new double[m, n];
                double[,] R = new double[n, n];

                // Modified Gram-Schmidt for better numerical stability
                for (int j = 0; j < n; j++)
                {
                    // Copy column j of A to column j of Q
                    for (int i = 0; i < m; i++)
                        Q[i, j] = A[i, j];

                    // Orthogonalize against previous columns
                    for (int k = 0; k < j; k++)
                    {
                        // R[k,j] = Q[:,k]^T * Q[:,j]
                        R[k, j] = 0;
                        for (int i = 0; i < m; i++)
                            R[k, j] += Q[i, k] * Q[i, j];

                        // Q[:,j] = Q[:,j] - R[k,j] * Q[:,k]
                        for (int i = 0; i < m; i++)
                            Q[i, j] -= R[k, j] * Q[i, k];
                    }

                    // Normalize Q[:,j]
                    double norm = 0;
                    for (int i = 0; i < m; i++)
                        norm += Q[i, j] * Q[i, j];
                    norm = Math.Sqrt(norm);

                    if (norm < 1e-10)
                        throw new InvalidOperationException("Matrix is rank deficient");

                    R[j, j] = norm;
                    for (int i = 0; i < m; i++)
                        Q[i, j] /= norm;
                }

                return new MyTuple<double[,], double[,]>(Q, R);
            }

            // ============================================================================
            // HELPER: Inverse of Upper Triangular Matrix (Fast!)
            // ============================================================================

            public static double[,] InverseUpperTriangular(double[,] R)
            {
                int n = R.GetLength(0);
                double[,] inv = new double[n, n];

                // Back substitution
                for (int j = n - 1; j >= 0; j--)
                {
                    inv[j, j] = 1.0 / R[j, j];
                    for (int i = j - 1; i >= 0; i--)
                    {
                        double sum = 0;
                        for (int k = i + 1; k <= j; k++)
                            sum += R[i, k] * inv[k, j];
                        inv[i, j] = -sum / R[i, i];
                    }
                }

                return inv;
            }

            // ============================================================================
            // BASIC MATRIX OPERATIONS
            // ============================================================================

            public static double[,] MultiplyMatrices(double[,] A, double[,] B)
            {
                int rowsA = A.GetLength(0);
                int colsA = A.GetLength(1);
                int colsB = B.GetLength(1);

                if (colsA != B.GetLength(0))
                    throw new ArgumentException("Matrix dimensions don't match for multiplication");

                double[,] result = new double[rowsA, colsB];

                for (int i = 0; i < rowsA; i++)
                {
                    for (int j = 0; j < colsB; j++)
                    {
                        double sum = 0;
                        for (int k = 0; k < colsA; k++)
                            sum += A[i, k] * B[k, j];
                        result[i, j] = sum;
                    }
                }

                return result;
            }

            public static double[,] MultiplyScalar(double[,] matrix, double scalar)
            {
                int rows = matrix.GetLength(0);
                int cols = matrix.GetLength(1);
                double[,] result = new double[rows, cols];
                for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                        result[i, j] = matrix[i, j] * scalar;
                return result;
            }

            public static double[] MultiplyScalar(double[] vector, double scalar)
            {
                int length = vector.Length;
                double[] result = new double[length];
                for (int i = 0; i < length; i++)
                    result[i] = vector[i] * scalar;
                return result;
            }

            public static double[,] AddMatrices(double[,] A, double[,] B)
            {
                int rows = A.GetLength(0);
                int cols = A.GetLength(1);
                if (rows != B.GetLength(0) || cols != B.GetLength(1))
                    throw new ArgumentException("Matrix dimensions don't match for addition");
                double[,] result = new double[rows, cols];
                for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                        result[i, j] = A[i, j] + B[i, j];
                return result;
            }

            public static double[,] Transpose(double[,] matrix)
            {
                int rows = matrix.GetLength(0);
                int cols = matrix.GetLength(1);
                double[,] result = new double[cols, rows];

                for (int i = 0; i < rows; i++)
                    for (int j = 0; j < cols; j++)
                        result[j, i] = matrix[i, j];

                return result;
            }

            // Column vector: result = matrix * vector
            public static double[] MultiplyMatrixVector(double[,] matrix, double[] vector)
            {
                int rows = matrix.GetLength(0);
                int cols = matrix.GetLength(1);

                if (cols != vector.Length)
                    throw new ArgumentException("Dimensions don't match");

                double[] result = new double[rows];
                for (int i = 0; i < rows; i++)
                {
                    double sum = 0;
                    for (int j = 0; j < cols; j++)
                        sum += matrix[i, j] * vector[j];
                    result[i] = sum;
                }

                return result;
            }

            // Row vector: result = vector * matrix
            public static double[] MultiplyVectorMatrix(double[] vector, double[,] matrix)
            {
                int rows = matrix.GetLength(0);
                int cols = matrix.GetLength(1);

                if (vector.Length != rows)
                    throw new ArgumentException("Dimensions don't match");

                double[] result = new double[cols];
                for (int j = 0; j < cols; j++)
                {
                    double sum = 0;
                    for (int i = 0; i < rows; i++)
                        sum += vector[i] * matrix[i, j];
                    result[j] = sum;
                }

                return result;
            }

            public static double[] MultiplyVectorVector(double[] a, double[] b)
            {
                if (a.Length != b.Length)
                    throw new ArgumentException("Vectors must be the same length");
                double[] result = new double[a.Length];
                for (int i = 0; i < a.Length; i++)
                    result[i] = a[i] * b[i];
                return result;
            }

            public static double DotProduct(double[] a, double[] b)
            {
                if (a.Length != b.Length)
                    throw new ArgumentException("Vectors must be the same length");
                double result = 0;
                for (int i = 0; i < a.Length; i++)
                    result += a[i] * b[i];
                return result;
            }

            public static double Norm(double[] vector)
            {
                double sum = 0;
                for (int i = 0; i < vector.Length; i++)
                {
                    sum += vector[i] * vector[i];
                }
                return Math.Sqrt(sum);
            }

            public static double[] GetRow(double[,] matrix, int rowIndex)
            {
                int cols = matrix.GetLength(1);
                double[] row = new double[cols];
                for (int j = 0; j < cols; j++)
                    row[j] = matrix[rowIndex, j];
                return row;
            }

            public static double[] GetColumn(double[,] matrix, int colIndex)
            {
                int rows = matrix.GetLength(0);
                double[] column = new double[rows];
                for (int i = 0; i < rows; i++)
                {
                    column[i] = matrix[i, colIndex];
                }
                return column;
            }
        }
    }
}
