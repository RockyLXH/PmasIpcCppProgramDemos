#ifndef _MAESTRO_LIB_H
#define _MAESTRO_LIB_H


#ifdef __cplusplus  
extern "C" {  // only need to export C interface if  
			  // used by C++ source code  
#endif

#define  _MAESTRO_LIB_H

/*!
 *
 * Sample Empty class for Elmo Maestro Library
 *	.
 */
class  Maestro_LibExample {

public:
	/*!
	 * constructor
	 */
	Maestro_LibExample();
	/*!
	 * destructor
	 */
	~Maestro_LibExample();

		/*!
	 * \fn int get_MaestroLibVersion();
	 * \brief return lirary version
	 *
	 */
	int get_MaestroLibVersion();
protected:

public:


private:
};

#ifdef __cplusplus  
}
#endif  

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif


#endif //_MAESTRO_LIB_H
