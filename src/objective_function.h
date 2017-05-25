/*
 * Copyright (c) 2011 Adrian Michel
 * http://www.amichel.com
 *
 * Permission to use, copy, modify, distribute and sell this 
 * software and its documentation for any purpose is hereby 
 * granted without fee, provided that both the above copyright 
 * notice and this permission notice appear in all copies and in 
 * the supporting documentation. 
 *  
 * This library is distributed in the hope that it will be 
 * useful. However, Adrian Michel makes no representations about
 * the suitability of this software for any purpose.  It is 
 * provided "as is" without any express or implied warranty. 
 * 
 * Should you find this library useful, please email 
 * info@amichel.com with a link or other reference 
 * to your work. 
*/

#ifndef DE_OBJECTIVE_FUNCTION_HPP_INCLUDED
#define DE_OBJECTIVE_FUNCTION_HPP_INCLUDED

// MS compatible compilers support #pragma once

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include <boost/shared_ptr.hpp>

#include <de_types.hpp>
#include <processors.hpp>


/**
 * Abstract base class for concrete objective functions. 
 *  
 * @author adrian (12/1/2011)
 */
class objective_function
{
private:
	const std::string m_name;
public:
	/**
	 * constructs an objective_function object
	 * 
	 * @author adrian (12/4/2011)
	 * 
	 * @param name the objective function name
	 */
	objective_function( const std::string& name )
	: m_name( name )
	{
	}

	virtual ~objective_function(){}

	/**
	 * Implemented in derived classes - it contains the objective 
	 * function to optimize. 
	 *  
	 * An objective function takes a vector of double arguments and 
	 * returns the calculated double value. 
	 *  
	 * Each index in the args vector corresponds to the same index 
	 * in the constraints vector. If for example the objective 
	 * function requires two variables, then set the constraints 
	 * vector's first two elements to the constraint for the two 
	 * variables, and in the objective function operator() use the 
	 * first two values of the args vector as the two variables. All
	 * the other values in this vector can be ignored.
	 * 
	 * @author adrian (12/1/2011)
	 * 
	 * @param args the vector of arguments. The vector is usually 
	 *  		   much larger than the number of variables used by
	 *  		   the objective function, so the OF will take only
	 *  		   the first n values in the vector, and ignore the
	 *  		   rest.
	 *  
	 * 
	 * 
	 * @return double the function cost which is the value that 
	 *  	   needs to be optimized
	 */
	virtual double operator()( de::DVectorPtr args ) = 0;

	/**
	 * An objective function has a name 
	 * 
	 * @author adrian (12/1/2011)
	 * 
	 * @return const std::string& 
	 */
	const std::string& name() const { return m_name; }
};

/**
 * Smart pointer to an objective function
 */
typedef boost::shared_ptr< objective_function > objective_function_ptr;


#endif //DE_OBJECTIVE_FUNCTION_HPP_INCLUDED

