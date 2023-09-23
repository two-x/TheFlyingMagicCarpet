
/* Contains unit tests */
#pragma once
#include <cassert>  // for assert
#include "utils.h"
#include <memory>

void test_Param() {
    // default constructor
    printf("Testing default constructor...\n");
    Param<int> t1;
    assert(t1.get_ptr().get() != t1.get_min_ptr().get());
    assert(t1.get_ptr().get() != t1.get_max_ptr().get());
    assert(t1.get_min_ptr().get() != t1.get_max_ptr().get());
    assert(t1.get_ptr().use_count() == 2); // 1 for Param + 1 for each external
    assert(t1.get_min_ptr().use_count() == 2); // 1 for Param + 1 for each external
    assert(t1.get_max_ptr().use_count() == 2); // 1 for Param + 1 for each external
    assert(t1.get() == t1.get_min());
    assert(t1.get_min() == t1.get_max());
    assert(t1.get_max() == t1.get_last());
    assert(t1.get() == t1.get_last());
    assert(t1.get_saturated() == false);
    // test set
    assert(!t1.set(3)); // value didn't change
    assert(t1.get() == t1.get_last());
    assert(t1.get_min() == t1.get_last());
    assert(t1.get_max() == t1.get_last());
    assert(t1.get_saturated() == true);
    // test add
    assert(!t1.add(1)); // value didn't change
    assert(t1.get() == t1.get_last());
    assert(t1.get_min() == t1.get_last());
    assert(t1.get_max() == t1.get_last());
    assert(t1.get_saturated() == true);
    // test set_limits
    t1.set_limits(1, 5);
    assert(t1.get_min() == 1);
    assert(t1.get_max() == 5);
    assert(t1.set(3)); // value changed
    assert(t1.get() == 3);
    assert(t1.get_saturated() == false);
    assert(t1.add(1));
    assert(t1.get() == 4);
    assert(t1.get_last() == 3);
    assert(t1.get_saturated() == false);

    // copy constructor
    printf("Testing copy constructor...\n");
    Param<int> t2(t1);
    assert(t1.get() == t2.get());
    assert(t1.get_min() == t2.get_min());
    assert(t1.get_max() == t2.get_max());
    assert(t1.get_ptr().get() == t2.get_ptr().get());
    assert(t1.get_min_ptr().get() == t2.get_min_ptr().get());
    assert(t1.get_max_ptr().get() == t2.get_max_ptr().get());
    assert(t1.get_ptr().use_count() == 3); // 1 for each Param + 1 external
    assert(t1.get_min_ptr().use_count() == 3); // 1 for each Param + 1 external
    assert(t1.get_max_ptr().use_count() == 3); // 1 for each Param + 1 external
    // test set
    t1.set(3);
    assert(t2.get() == 3);
    // test add
    t1.add(1);
    assert(t2.get() == 4);
    // test set_limits
    t1.set_limits(0,15);
    assert(t2.get() == 4);
    assert(t2.get_min() == 0);
    assert(t2.get_max() == 15);
    assert(t2.get_saturated() == false);

    // destructor
    printf("Testing destructor...\n");
    std::shared_ptr<int> r1;
    Param<int> *
    
    ;
    {
        Param<int> p2;
        p1 = new Param<int>(p2);
        r1 = p2.get_ptr();
        assert(p2.get_ptr().use_count() == 4); // 1 for each Param + 2 external
    }
    assert(p1->get_ptr().use_count() == 3); // 1 for Param + 2 external
    delete p1;
    assert(r1.use_count() == 1); // 1 external

    // value constructor
    printf("Testing value constructor...\n");
    Param<int> t3(13);
    assert(t3.get() == 13);
    assert(t3.get() == t3.get_min());
    assert(t3.get() == t3.get_max());
    assert(t3.get() == t3.get_last());
    assert(t3.get_ptr().use_count() == 2);
    assert(t3.get_min_ptr().use_count() == 2);
    assert(t3.get_max_ptr().use_count() == 2); // 1 for Param + 1 external
    assert(t3.get_ptr().get() != t3.get_min_ptr().get());
    assert(t3.get_ptr().get() != t3.get_max_ptr().get());
    assert(t3.get_min_ptr().get() != t3.get_max_ptr().get());

    // set_limits
    printf("Testing set_limits...\n");
    // test set (below limit)
    assert(!t3.set(3)); // value didn't change
    assert(t3.get() == 13);
    assert(t3.get_min() == 13);
    assert(t3.get_max() == 13);
    assert(t3.get_saturated() == true);
    // test add (above limit)
    assert(!t3.add(1)); // value didn't change
    assert(t3.get() == 13);
    assert(t3.get_min() == 13);
    assert(t3.get_max() == 13);
    assert(t3.get_saturated() == true);
    // test tight set_limits + upwards constrain
    t3.set_limits(15,15);
    assert(t3.get() == 15);
    assert(t3.get_min() == 15);
    assert(t3.get_max() == 15);
    assert(t3.get_saturated() == true);
    // test tight set_limits + downwards constrain
    t3.set_limits(7,7);
    assert(t3.get() == 7);
    assert(t3.get_min() == 7);
    assert(t3.get_max() == 7);
    assert(t3.get_saturated() == true);
    // test set_limits
    t3.set_limits(5,15);
    assert(t3.get() == 7);
    assert(t3.get_min() == 5);
    assert(t3.get_max() == 15);
    assert(t3.get_saturated() == false);
    // test set with limits
    t3.set(8);
    assert(t3.get() == 8);
    assert(t3.get_min() == 5);
    assert(t3.get_max() == 15);
    assert(t3.get_saturated() == false);
    // test add with limits
    t3.add(1);
    assert(t3.get() == 9);
    assert(t3.get_min() == 5);
    assert(t3.get_max() == 15);
    assert(t3.get_saturated() == false);
    // test set_limits + downwards constrain
    t3.set_limits(10,15);
    assert(t3.get() == 10);
    assert(t3.get_min() == 10);
    assert(t3.get_max() == 15);
    assert(t3.get_saturated() == true);
    // test set_limits + upwards constrain
    t3.set_limits(2,8);
    assert(t3.get() == 8);
    assert(t3.get_min() == 2);
    assert(t3.get_max() == 8);
    assert(t3.get_saturated() == true);

    // limits constructor
    printf("Testing limits constructor...\n");
    Param<int> t4(5, 1, 10);
    assert(t4.get() == 5);
    assert(t4.get_min() == 1);
    assert(t4.get_max() == 10);
    assert(t4.get_last() == 5);
    assert(t4.get_saturated() == false);
    assert(t4.get_ptr().use_count() == 2);
    assert(t4.get_min_ptr().use_count() == 2);
    assert(t4.get_max_ptr().use_count() == 2); // 1 for Param + 1 external
    assert(t4.get_ptr().get() != t4.get_min_ptr().get());
    assert(t4.get_ptr().get() != t4.get_max_ptr().get());
    assert(t4.get_min_ptr().get() != t4.get_max_ptr().get());
    // test constructor with upwards constrain
    Param<int> t5(1, 5, 10);
    assert(t5.get() == 5);
    assert(t5.get_min() == 5);
    assert(t5.get_max() == 10);
    assert(t5.get_last() == 5);
    assert(t5.get_saturated() == true);
    // test constructor with downwards constrain
    Param<int> t6(10, 1, 5);
    assert(t6.get() == 5);
    assert(t6.get_min() == 1);
    assert(t6.get_max() == 5);
    assert(t6.get_last() == 5);
    assert(t6.get_saturated() == true);

    // external limits constructor
    printf("Testing external limits constructor...\n");
    Param<int> t7(5, 0, 20);
    Param<int> t8(10, 0, 20);
    Param<int> t9(8, t7.get_ptr(), t8.get_ptr());
    assert(t9.get() == 8);
    assert(t9.get_min() == 5);
    assert(t9.get_max() == 10);
    assert(t9.get_last() == 8);
    assert(t9.get_saturated() == false);
    assert(t9.get_ptr().use_count() == 2);
    assert(t9.get_min_ptr().use_count() == 3); // 1 for each Param + 1 external
    assert(t9.get_max_ptr().use_count() == 3); // 1 for each Param + 1 external
    assert(t7.get_ptr().get() == t9.get_min_ptr().get());
    assert(t8.get_ptr().get() == t9.get_max_ptr().get());
    // test constructor with upwards constrain
    Param<int> t10(1, t7.get_ptr(), t8.get_ptr());
    assert(t10.get() == 5);
    assert(t10.get_min() == 5);
    assert(t10.get_max() == 10);
    assert(t10.get_last() == 5);
    assert(t10.get_saturated() == true);
    // test constructor with downwards constrain
    Param<int> t11(11, t7.get_ptr(), t8.get_ptr());
    assert(t11.get() == 10);
    assert(t11.get_min() == 5);
    assert(t11.get_max() == 10);
    assert(t11.get_last() == 10);
    assert(t11.get_saturated() == true);
    // test change limits
    Param<int> t12(8, t7.get_ptr(), t8.get_ptr());
    t7.set(4);
    t8.set(11);
    assert(t12.get() == 8);
    assert(t12.get_min() == 4);
    assert(t12.get_max() == 11);
    assert(t12.get_last() == 8);
    assert(t12.get_saturated() == false);
    // test change limits with upwards constrain
    t7.set(9);
    assert(t12.get() == 8);
    assert(t12.get_min() == 9);
    assert(t12.get_max() == 11);
    assert(t12.get_last() == 8);
    assert(t12.get_saturated() == false); // NOTE: should we constrain on get()...?
    t12.set(8);
    assert(t12.get() == 9);
    assert(t12.get_min() == 9);
    assert(t12.get_max() == 11);
    assert(t12.get_last() == 8);
    assert(t12.get_saturated() == true); // NOTE: should we constrain on get()...?
    // test change limits with downwards constrain
    t12.set(10);
    t7.set(1);
    t8.set(7);
    assert(t12.get() == 10);
    assert(t12.get_min() == 1);
    assert(t12.get_max() == 7);
    assert(t12.get_last() == 9);
    assert(t12.get_saturated() == false); // NOTE: should we constrain on get()...?
    t12.set(9);
    assert(t12.get() == 7);
    assert(t12.get_min() == 1);
    assert(t12.get_max() == 7);
    assert(t12.get_last() == 10);
    assert(t12.get_saturated() == true); // NOTE: should we constrain on get()...?
    // test external limits destructor
    Param<int> t13(10);
    {
        Param<int> p1(5);
        Param<int> p2(15);
        t13.set_limits(p1.get_ptr(), p2.get_ptr());
    }
    assert(t13.get_min() == 5);
    assert(t13.get_max() == 15);
    assert(t13.get_min_ptr().use_count() == 2);
    assert(t13.get_max_ptr().use_count() == 2);
}