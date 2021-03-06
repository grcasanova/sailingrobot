/****************************************************************************************
 *
 * File:
 * 		ASRCourseBallot.h
 *
 * Purpose:
 *		The Course Ballot is used by the voters and the Arbiter to store the possible 
 *      course headings and their votes. It provides functions for setting and clearing 
 *      the voting scores, as well as accessing the underlying structure. 
 *
 * License:
 *      This file is subject to the terms and conditions defined in the file 
 *      'LICENSE.txt', which is part of this source code package.
 *
 ***************************************************************************************/

#pragma once


#include <stdint.h>
#include <cstring>


class ASRCourseBallot {
    friend class ASRArbiter; 
public:
    ///----------------------------------------------------------------------------------
 	/// Constructs a CourseBallot. Requires a single argument which controls the maximum
    /// vote a single course can have.
 	///----------------------------------------------------------------------------------
    ASRCourseBallot( int16_t maxVotes );

    ///----------------------------------------------------------------------------------
 	/// Assigns a vote to a particular heading.
 	///----------------------------------------------------------------------------------
    void set( uint16_t course, int16_t value );

    ///----------------------------------------------------------------------------------
 	/// Adds a vote to a particular heading.
 	///----------------------------------------------------------------------------------
    void add( uint16_t course, int16_t value );

    ///----------------------------------------------------------------------------------
 	/// Resets the ballot, clearing all set votes.
 	///----------------------------------------------------------------------------------
    void clear();

    ///----------------------------------------------------------------------------------
 	/// Gets the votes placed on a heading. The heading is rounded down to the nearest 
    /// valid heading;
 	///----------------------------------------------------------------------------------
    int16_t get( uint16_t heading ) const;

    ///----------------------------------------------------------------------------------
 	/// Returns a pointer to the underlying course data, this is an array that has
    /// 360 / CourseBallot::COURSE_RESOLUTION elements.
 	///----------------------------------------------------------------------------------
    const int16_t* ptr() const;

    const int16_t maxVotes() { return MAX_VOTES; }

    ///----------------------------------------------------------------------------------
 	/// The course resolution controls the number of courses that the boat will examine.
    /// The number of courses examined is 360 / CourseBallot::COURSE_RESOLUTION. This 
    /// should be some value that 360 is divisible by without resulting in a fraction.
 	///----------------------------------------------------------------------------------
    static const int COURSE_RESOLUTION = 1;

    ///----------------------------------------------------------------------------------
 	/// The number of courses the ballot tracks.
 	///----------------------------------------------------------------------------------
    static const int ELEMENT_COUNT = 360 / COURSE_RESOLUTION;
private:

    const int16_t MAX_VOTES;
    int16_t courses[ELEMENT_COUNT];
};