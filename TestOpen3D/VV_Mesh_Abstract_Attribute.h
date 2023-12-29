#pragma once

#include <vector>

template<class T>
class VV_Mesh_Abstract_Attribute
{
	std::vector<T> attributes;

public:
	size_t GetAttributeCount() { return attributes.size(); }

	T GetAttribute(int index) { return attributes[index]; }

	void Resize(size_t new_size) { attributes.resize(new_size); }

	void Set(int index, T value) { attributes[index] = value; }
};