/*
 * MIPP is distributed under the MIT License.
 * See mipp.h for the full license text bundled with this directory.
 */

/**
 * @file mipp_scalar_op.h
 * @brief MIPP 标量 fallback 运算声明。
 * @details 当 SIMD intrinsic 不可用或需要标量路径时，MIPP 使用这些模板函数
 *          复刻向量操作语义。当前工程仅补文档，不改变函数签名或行为。
 */

#ifndef MIPP_SCALAR_OP_H_
#define MIPP_SCALAR_OP_H_

/**
 * @brief MIPP 标量操作命名空间。
 *
 * SCOP 表示 SCalar OPerations，用于模拟 add/sub/bit shift 等 SIMD lane 操作。
 */
namespace mipp_scop // My Intrinsics Plus Plus SCalar OPerations
{
	/// @brief 标量加法，窄整数特化会执行饱和保护。
	template <typename T>
	inline T add(const T val1, const T val2);

	/// @brief 标量减法，窄整数特化会执行饱和保护。
	template <typename T>
	inline T sub(const T val1, const T val2);

	/// @brief 标量按位与。
	template <typename T>
	inline T andb(const T val1, const T val2);

	/// @brief 标量按位异或。
	template <typename T>
	inline T xorb(const T val1, const T val2);

	/// @brief 提取最高有效位。
	template <typename T>
	inline T msb(const T val);

	/// @brief 除以 2；整数特化使用右移。
	template <typename T>
	inline T div2(const T val);

	/// @brief 除以 4；整数特化使用右移。
	template <typename T>
	inline T div4(const T val);

	/// @brief 右移 n 位。
	template <typename T>
	inline T rshift(const T val, const int n);

	/// @brief 左移 n 位。
	template <typename T>
	inline T lshift(const T val, const int n);
}

#include "mipp_scalar_op.hxx"

#endif /* MIPP_SCALAR_OP_H_ */
