#ifndef X__LED__H__
#define X__LED__H__


namespace Led{

	enum _RedState{ _On, _Off, _1Hz, _3Hz };
	void Init();
	void Module();
	void Red( _RedState state );
}

#endif
