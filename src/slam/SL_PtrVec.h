/*
 * SL_PtrVec.h
 *
 *  Created on: 2010-11-6
 *      Author: Danping Zou
 *		E-mail: dannis.zou@gmail.com
 */

#ifndef SL_PTRVEC_H_
#define SL_PTRVEC_H_

/* class for automatically release the allocated pointer memories*/
template<typename T>
class PtrVec {
public:
	T** data;
	int len;
public:
	typedef T* PT;
	PtrVec() :
		data(0), len(0) {
	}
	;
	virtual ~PtrVec() {
		clear();
	}
public:
	int size() const {
		return len;
	}
	void reserve(int num) {
		clear();
		data = new T*[num];
	}
	void push_back(T* ptr) {
		data[len++] = ptr;
	}
	bool empty() const {
		return len <= 0;
	}
	void clear() {
		if (empty())
			return;
		else { /* release the element memory */
			int i;
			for (i = 0; i < len; ++i) {
				delete data[i];
			}
		}
		/*release the pointer's memory*/
		delete[] data;
		data = 0;
		len = 0;
	}
	inline T* operator[](int i) const {
		return data[i];
	}
	inline PT& operator[](int i) {
		return data[i];
	}
};
#endif /* SL_PTRVEC_H_ */
