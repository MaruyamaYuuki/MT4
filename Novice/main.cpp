#include <Novice.h>
#include <iostream>
#include"math/Matrix4x4.h"
#include"math/Vector3.h"
#include "math/MathUtility.h"
#include <assert.h>

using namespace KamataEngine;

const char kWindowTitle[] = "LE2C_24_マルヤマ_ユウキ";

KamataEngine::Matrix4x4 MakeRotateAxisAngle(const KamataEngine::Vector3& axis, float angle) {
	KamataEngine::Matrix4x4 result = KamataEngine::MathUtility::MakeIdentityMatrix();
	result.m[0][0] = (axis.x * axis.x) * (1 - std::cos(angle)) + std::cos(angle);
	result.m[0][1] = axis.x * (axis.y * (1 - std::cos(angle))) + (axis.z * std::sin(angle));
	result.m[0][2] = axis.x * (axis.z * (1 - std::cos(angle))) - (axis.y * std::sin(angle));

	result.m[1][0] = axis.x * (axis.y * (1 - std::cos(angle))) - (axis.z * std::sin(angle));
	result.m[1][1] = (axis.y * axis.y) * (1 - std::cos(angle)) + std::cos(angle);
	result.m[1][2] = axis.y * (axis.z * (1 - std::cos(angle))) + (axis.x * std::sin(angle));

	result.m[2][0] = axis.x * (axis.z * (1 - std::cos(angle))) + (axis.y * std::sin(angle));
	result.m[2][1] = axis.y * (axis.z * (1 - std::cos(angle))) - (axis.x * std::sin(angle));
	result.m[2][2] = (axis.z * axis.z) * (1 - std::cos(angle)) + std::cos(angle);
	return result;
}

KamataEngine::Matrix4x4 DirectionToDiretion(const KamataEngine::Vector3& from, const KamataEngine::Vector3& to) {

	KamataEngine::Vector3 num = KamataEngine::MathUtility::Cross(from, to);
	float cos = KamataEngine::MathUtility::Dot(from, to);
	float sin = KamataEngine::MathUtility::Length(num);

	float epsilon = 1e-6f;
	KamataEngine::Vector3 axis = {};

	if (std::abs(cos + 1.0f) <= epsilon) {
		// 反対方向のベクトルに回転する場合の処理
		if (std::abs(from.x) > epsilon || std::abs(from.y) > epsilon) {
			// (ux≠0||uy≠0) の際の axis の値を入れる 
			axis.x = from.y;
			axis.y = -from.x;
			axis.z = 0.0f;
		}
		else if (std::abs(from.x) > epsilon || std::abs(from.z) > epsilon) {
			// (ux≠0||uz≠0) の際の axis の値を入れる 
			axis.x = from.z;
			axis.y = 0.0f;
			axis.z = -from.x;
		}
		else {
			// zero vector 
			assert(false);
		}
	}
	else {
		// それ以外の通常のケース
		axis = KamataEngine::MathUtility::Normalize(num);
	}

	axis = KamataEngine::MathUtility::Normalize(axis);

	KamataEngine::Matrix4x4 result = KamataEngine::MathUtility::MakeIdentityMatrix();
	result.m[0][0] = (axis.x * axis.x) * (1 - cos) + cos;
	result.m[0][1] = axis.x * (axis.y * (1 - cos)) + (axis.z * sin);
	result.m[0][2] = axis.x * (axis.z * (1 - cos)) - (axis.y * sin);

	result.m[1][0] = axis.x * (axis.y * (1 - cos)) - (axis.z * sin);
	result.m[1][1] = (axis.y * axis.y) * (1 - cos) + cos;
	result.m[1][2] = axis.y * (axis.z * (1 - cos)) + (axis.x * sin);

	result.m[2][0] = axis.x * (axis.z * (1 - cos)) + (axis.y * sin);
	result.m[2][1] = axis.y * (axis.z * (1 - cos)) - (axis.x * sin);
	result.m[2][2] = (axis.z * axis.z) * (1 - cos) + cos;
	return result;
}

static const int kRowHeight = 20;
static const int kColumnWidth = 60;
void MatrixScreenPrint(int x, int y, const KamataEngine::Matrix4x4& matrix, const char* label) {
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

	Vector3 fromNum0 = { 1.0f,0.7f,0.5f };
	Vector3 fromNum1 = { -0.6f,0.9f,0.2f };
	Vector3 toNum1 = { 0.4f,0.7f,-0.5f };
	Vector3 num1 = { 1.0f,0.0f,0.0f };
	Vector3 num2 = { -1.0f,0.0f,0.0f };

	Vector3 from0 = MathUtility::Normalize(fromNum0);
	Vector3 to0; to0.x = -from0.x; to0.y = -from0.y; to0.z = -from0.z;
	Vector3 from1 = MathUtility::Normalize(fromNum1);
	Vector3 to1 = MathUtility::Normalize(toNum1);

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

		Matrix4x4 rotateMatrix0 = DirectionToDiretion(MathUtility::Normalize(num1), MathUtility::Normalize(num2));
		Matrix4x4 rotateMatrix1 = DirectionToDiretion(from0, to0);
		Matrix4x4 rotateMatrix2 = DirectionToDiretion(from1, to1);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		MatrixScreenPrint(0, 0, rotateMatrix0, "rotateMatrix0");
		MatrixScreenPrint(0, kRowHeight * 5, rotateMatrix1, "rotateMatrix1");
		MatrixScreenPrint(0, kRowHeight * 10, rotateMatrix2, "rotateMatrix2");

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
