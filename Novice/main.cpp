#include <Novice.h>
#include <iostream>
#include"math/Matrix4x4.h"
#include"math/Vector3.h"
#include "math/MathUtility.h"

const char kWindowTitle[] = "LE2C_24_マルヤマ_ユウキ";
KamataEngine::Matrix4x4 MakeRotateAxisAngle(const KamataEngine::Vector3& axise, float angle) {
	KamataEngine::Matrix4x4 result = KamataEngine::MathUtility::MakeIdentityMatrix();
	result.m[0][0] = (axise.x * axise.x) * (1 - std::cos(angle)) + std::cos(angle);
	result.m[0][1] = axise.x * (axise.y * (1 - std::cos(angle))) + (axise.z * std::sin(angle));
	result.m[0][2] = axise.x * (axise.z * (1 - std::cos(angle))) - (axise.y * std::sin(angle));

	result.m[1][0] = axise.x * (axise.y * (1 - std::cos(angle))) - (axise.z * std::sin(angle));
	result.m[1][1] = (axise.y * axise.y) * (1 - std::cos(angle)) + std::cos(angle);
	result.m[1][2] = axise.y * (axise.z * (1 - std::cos(angle))) + (axise.x * std::sin(angle));

	result.m[2][0] = axise.x * (axise.z * (1 - std::cos(angle))) + (axise.y * std::sin(angle));
	result.m[2][1] = axise.y * (axise.z * (1 - std::cos(angle))) - (axise.x * std::sin(angle));
	result.m[2][2] = (axise.z * axise.z) * (1 - std::cos(angle)) + std::cos(angle);
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
	KamataEngine::Vector3 num = { 1.0f,1.0f,1.0f };
	KamataEngine::Vector3 axis = KamataEngine::MathUtility::Normalize(num);
	float angle = 0.44f;

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

		KamataEngine::Matrix4x4 rotateMatrix = MakeRotateAxisAngle(axis, angle);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		MatrixScreenPrint(0, 0, rotateMatrix, "rotateMatrix");

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
