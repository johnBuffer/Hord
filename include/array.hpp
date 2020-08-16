#pragma once

template<typename T, uint32_t N>
struct Array
{
	T& operator[](const uint32_t index) {
		return data[index];
	}

	T operator[](const uint32_t index) const {
		return data[index];
	}

	T data[N];
};
