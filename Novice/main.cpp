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
Quaternion Normalize(const Quaternion& quaternion) {
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

static const int kRowHeight = 20;
static const int kColumnWidth = 60;
void QuaternionScreenPrint(int x, int y, const Quaternion& quaternion, const char* ladel) {
	Novice::ScreenPrintf(x, y, "%.02f", quaternion.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", quaternion.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", quaternion.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%.02f", quaternion.w);
	Novice::ScreenPrintf(x + kColumnWidth * 4, y, "%s", ladel);
}

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	Quaternion q1 = { 2.0f,3.0f,4.0f,1.0f };
	Quaternion q2 = { 1.0f,3.0f,5.0f,2.0f };

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

		Quaternion identity = IdentityQuaternion();
		Quaternion conj = Conjugate(q1);
		Quaternion inv = Inverse(q1);
		Quaternion normal = Normalize(q1);
		Quaternion mul1 = Multiply(q1, q2);
		Quaternion mul2 = Multiply(q2, q1);
		float norm = Norm(q1);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		QuaternionScreenPrint(0, 0, identity, " : Identity");
		QuaternionScreenPrint(0, kRowHeight, conj, " : Conjugate");
		QuaternionScreenPrint(0, kRowHeight * 2, inv, " : Inverse");
		QuaternionScreenPrint(0, kRowHeight * 3, normal, " : Normal");
		QuaternionScreenPrint(0, kRowHeight * 4, mul1, " : Multiply(q1,q2)");
		QuaternionScreenPrint(0, kRowHeight * 5, mul2, " : Multiply(q2,q1)");
		Novice::ScreenPrintf(0, kRowHeight * 6, "%.02f", norm);
		Novice::ScreenPrintf(kColumnWidth * 4, kRowHeight * 6, " : Norm");

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
