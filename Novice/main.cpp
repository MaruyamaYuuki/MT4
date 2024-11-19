#include <Novice.h>
#include <iostream>
#include"math/Matrix4x4.h"
#include"math/Vector3.h"
#include "math/MathUtility.h"
#include <assert.h>

using namespace KamataEngine;

const char kWindowTitle[] = "LE2C_24_マルヤマ_ユウキ";

struct Quaternion {
	float x;
	float y;
	float z;
	float w;
};

// Quaternionの積
Quaternion Multiply(const Quaternion& lhs, const Quaternion& rhs) {
	Quaternion result;
	result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
	result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
	result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
	result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
	return result;
}

// 単位Quaternionを返す 
Quaternion IdentityQuaternion() {
	Quaternion result = { 0.0f, 0.0f, 0.0f, 1.0f };
	return result;
}

// 共役Quaternionを返す 
Quaternion Conjugate(const Quaternion& quaternion) {
	Quaternion result = { -quaternion.x, -quaternion.y, -quaternion.z, quaternion.w };
	return result;
}

// Quaternionのnormを返す 
float Norm(const Quaternion& quaternion) {
	float result = std::sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x +
		quaternion.y * quaternion.y + quaternion.z * quaternion.z);
	return result;
}

// 正規化したQuaternionを返す 
Quaternion NormalizeQuaternion(const Quaternion& quaternion) {
	float norm = Norm(quaternion);
	Quaternion result;

	if (norm == 0.0f) {
		result = IdentityQuaternion();
	}
	else {
		result = { quaternion.x / norm, quaternion.y / norm, quaternion.z / norm, quaternion.w / norm };
	}

	return result;
}

// 逆Quaternionを返す 
Quaternion Inverse(const Quaternion& quaternion) {
	float normSquared = quaternion.w * quaternion.w + quaternion.x * quaternion.x +
		quaternion.y * quaternion.y + quaternion.z * quaternion.z;
	Quaternion result;

	if (normSquared == 0.0f) {
		result = IdentityQuaternion();
	}
	else {
		Quaternion conjugate = Conjugate(quaternion);
		result = { conjugate.x / normSquared, conjugate.y / normSquared,
				  conjugate.z / normSquared, conjugate.w / normSquared };
	}

	return result;
}

// 任意軸回転を表すQuaternionの生成
Quaternion MakeRotateAxisAngleQuaternion(const Vector3& axis, float angle) {
	Quaternion result;
	// 回転角度の半分を計算
	float halfAngle = angle * 0.5f;

	// sin(半分の角度) と cos(半分の角度) を計算
	float sinHalfAngle = sinf(halfAngle);
	float cosHalfAngle = cosf(halfAngle);

	// クォータニオンの成分を計算
	result.x = axis.x * sinHalfAngle;
	result.y = axis.y * sinHalfAngle;
	result.z = axis.z * sinHalfAngle;
	result.w = cosHalfAngle;

	return result;
}


// ベクトルをQuaternionで回転させた結果のベクトルを求める
Vector3 RotateVector(const Vector3& vector, const Quaternion& quaternion) {

	Quaternion conjugate = { -quaternion.x, -quaternion.y, -quaternion.z, quaternion.w };

	Quaternion qVector = { vector.x, vector.y, vector.z, 0.0f };

	Quaternion rotated = Multiply(Multiply(quaternion, qVector), conjugate);

	return { rotated.x, rotated.y, rotated.z };
}

// Quaternionから回転行列を求める
Matrix4x4 MakeRotateMatrix(const Quaternion& quaternion) {
	KamataEngine::Matrix4x4 matrix;

	float xx = quaternion.x * quaternion.x;
	float yy = quaternion.y * quaternion.y;
	float zz = quaternion.z * quaternion.z;
	float xy = quaternion.x * quaternion.y;
	float xz = quaternion.x * quaternion.z;
	float yz = quaternion.y * quaternion.z;
	float wx = quaternion.w * quaternion.x;
	float wy = quaternion.w * quaternion.y;
	float wz = quaternion.w * quaternion.z;

	matrix.m[0][0] = 1.0f - 2.0f * (yy + zz);
	matrix.m[0][1] = 2.0f * (xy + wz);
	matrix.m[0][2] = 2.0f * (xz - wy);
	matrix.m[0][3] = 0.0f;

	matrix.m[1][0] = 2.0f * (xy - wz);
	matrix.m[1][1] = 1.0f - 2.0f * (xx + zz);
	matrix.m[1][2] = 2.0f * (yz + wx);
	matrix.m[1][3] = 0.0f;

	matrix.m[2][0] = 2.0f * (xz + wy);
	matrix.m[2][1] = 2.0f * (yz - wx);
	matrix.m[2][2] = 1.0f - 2.0f * (xx + yy);
	matrix.m[2][3] = 0.0f;

	matrix.m[3][0] = 0.0f;
	matrix.m[3][1] = 0.0f;
	matrix.m[3][2] = 0.0f;
	matrix.m[3][3] = 1.0f;

	return matrix;
}

static const int kRowHeight = 20;
static const int kColumnWidth = 60;
void QuaternionScreenPrint(int x, int y, const Quaternion& quaternion, const char* ladel) {
	Novice::ScreenPrintf(x, y, "%.02f", quaternion.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", quaternion.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", quaternion.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%.02f", quaternion.w);
	Novice::ScreenPrintf(x + kColumnWidth * 4, y, "%s", ladel);
}

void VectorScreenPrint(int x, int y, const Vector3& vector, const char* ladel) {
	Novice::ScreenPrintf(x, y, "%.02f", vector.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", vector.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", vector.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%s", ladel);
}

void MatrixScreenPrint(int x, int y, const Matrix4x4& matrix, const char* label) {
	Novice::ScreenPrintf(x, y, "%s", label);
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(x + column * kColumnWidth, y + row * kRowHeight + kRowHeight, "%6.03f", matrix.m[row][column]);
		}
	}
}

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	Vector3 vector = { 1.0f,0.4f,-0.2f };
	Vector3 pointY = { 2.1f,-0.9f,1.3f };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		Quaternion rotation = MakeRotateAxisAngleQuaternion(
			MathUtility::Normalize(vector), 0.45f);
		Matrix4x4 rotateMatrix = MakeRotateMatrix(rotation);
		Vector3 rotateByQuaternion = RotateVector(pointY, rotation);
		Vector3 rotateByMatrix = MathUtility::Transform(pointY, rotateMatrix);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		QuaternionScreenPrint(0, kRowHeight * 0, rotation, " : rotation");
		MatrixScreenPrint(0, kRowHeight * 1, rotateMatrix, "rotateMatrix");
		VectorScreenPrint(0, kRowHeight * 6, rotateByQuaternion, " : rotatebyQuaternion");
		VectorScreenPrint(0, kRowHeight * 7, rotateByMatrix, " : rotateByMatrix");

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
