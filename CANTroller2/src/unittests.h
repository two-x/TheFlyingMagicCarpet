
/* Contains unit tests */
#pragma once
#include <cassert>  // for assert
#include "sensors.h"
#include <memory>

void test_Param() {
    // default constructor
    printf("Testing default constructor...\n");
    Param<int> t1;
    assert(t1.ptr() != t1.min_ptr());
    assert(t1.ptr() != t1.max_ptr());
    assert(t1.min_ptr() != t1.max_ptr());
    assert(t1.shptr().use_count() == 2); // 1 for Param + 1 for each external
    assert(t1.min_shptr().use_count() == 2); // 1 for Param + 1 for each external
    assert(t1.max_shptr().use_count() == 2); // 1 for Param + 1 for each external
    assert(t1.val() == t1.min());
    assert(t1.min() == t1.max());
    assert(t1.max() == t1.last());
    assert(t1.val() == t1.last());
    assert(t1.saturated() == false);
    // test set
    assert(!t1.set(3)); // value didn't change
    assert(t1.val() == t1.last());
    assert(t1.min() == t1.last());
    assert(t1.max() == t1.last());
    assert(t1.saturated() == true);
    // test add
    assert(!t1.add(1)); // value didn't change
    assert(t1.val() == t1.last());
    assert(t1.min() == t1.last());
    assert(t1.max() == t1.last());
    assert(t1.saturated() == true);
    // test set_limits
    t1.set_limits(1, 5);
    assert(t1.min() == 1);
    assert(t1.max() == 5);
    assert(t1.set(3)); // value changed
    assert(t1.val() == 3);
    assert(t1.saturated() == false);
    assert(t1.add(1));
    assert(t1.val() == 4);
    assert(t1.last() == 3);
    assert(t1.saturated() == false);

    // copy constructor
    printf("Testing copy constructor...\n");
    Param<int> t2(t1);
    assert(t1.val() == t2.val());
    assert(t1.min() == t2.min());
    assert(t1.max() == t2.max());
    assert(t1.ptr() == t2.ptr());
    assert(t1.min_ptr() == t2.min_ptr());
    assert(t1.max_ptr() == t2.max_ptr());
    assert(t1.shptr().use_count() == 3); // 1 for each Param + 1 external
    assert(t1.min_shptr().use_count() == 3); // 1 for each Param + 1 external
    assert(t1.max_shptr().use_count() == 3); // 1 for each Param + 1 external
    // test set
    t1.set(3);
    assert(t2.val() == 3);
    // test add
    t1.add(1);
    assert(t2.val() == 4);
    // test set_limits
    t1.set_limits(0,15);
    assert(t2.val() == 4);
    assert(t2.min() == 0);
    assert(t2.max() == 15);
    assert(t2.saturated() == false);

    // destructor
    printf("Testing destructor...\n");
    std::shared_ptr<int> r1;
    Param<int> *p1;
    {
        Param<int> p2;
        p1 = new Param<int>(p2);
        r1 = p2.shptr();
        assert(p2.shptr().use_count() == 4); // 1 for each Param + 2 external
    }
    assert(p1->shptr().use_count() == 3); // 1 for Param + 2 external
    delete p1;
    assert(r1.use_count() == 1); // 1 external

    // value constructor
    printf("Testing value constructor...\n");
    Param<int> t3(13);
    assert(t3.val() == 13);
    assert(t3.val() == t3.min());
    assert(t3.val() == t3.max());
    assert(t3.val() == t3.last());
    assert(t3.shptr().use_count() == 2);
    assert(t3.min_shptr().use_count() == 2);
    assert(t3.max_shptr().use_count() == 2); // 1 for Param + 1 external
    assert(t3.ptr() != t3.min_ptr());
    assert(t3.ptr() != t3.max_ptr());
    assert(t3.min_ptr() != t3.max_ptr());

    // set_limits
    printf("Testing set_limits...\n");
    // test set (below limit)
    assert(!t3.set(3)); // value didn't change
    assert(t3.val() == 13);
    assert(t3.min() == 13);
    assert(t3.max() == 13);
    assert(t3.saturated() == true);
    // test add (above limit)
    assert(!t3.add(1)); // value didn't change
    assert(t3.val() == 13);
    assert(t3.min() == 13);
    assert(t3.max() == 13);
    assert(t3.saturated() == true);
    // test tight set_limits + upwards constrain
    t3.set_limits(15,15);
    assert(t3.val() == 15);
    assert(t3.min() == 15);
    assert(t3.max() == 15);
    assert(t3.saturated() == true);
    // test tight set_limits + downwards constrain
    t3.set_limits(7,7);
    assert(t3.val() == 7);
    assert(t3.min() == 7);
    assert(t3.max() == 7);
    assert(t3.saturated() == true);
    // test set_limits
    t3.set_limits(5,15);
    assert(t3.val() == 7);
    assert(t3.min() == 5);
    assert(t3.max() == 15);
    assert(t3.saturated() == false);
    // test set with limits
    t3.set(8);
    assert(t3.val() == 8);
    assert(t3.min() == 5);
    assert(t3.max() == 15);
    assert(t3.saturated() == false);
    // test add with limits
    t3.add(1);
    assert(t3.val() == 9);
    assert(t3.min() == 5);
    assert(t3.max() == 15);
    assert(t3.saturated() == false);
    // test set_limits + downwards constrain
    t3.set_limits(10,15);
    assert(t3.val() == 10);
    assert(t3.min() == 10);
    assert(t3.max() == 15);
    assert(t3.saturated() == true);
    // test set_limits + upwards constrain
    t3.set_limits(2,8);
    assert(t3.val() == 8);
    assert(t3.min() == 2);
    assert(t3.max() == 8);
    assert(t3.saturated() == true);

    // limits constructor
    printf("Testing limits constructor...\n");
    Param<int> t4(5, 1, 10);
    assert(t4.val() == 5);
    assert(t4.min() == 1);
    assert(t4.max() == 10);
    assert(t4.last() == 5);
    assert(t4.saturated() == false);
    assert(t4.shptr().use_count() == 2);
    assert(t4.min_shptr().use_count() == 2);
    assert(t4.max_shptr().use_count() == 2); // 1 for Param + 1 external
    assert(t4.ptr() != t4.min_ptr());
    assert(t4.ptr() != t4.max_ptr());
    assert(t4.min_ptr() != t4.max_ptr());
    // test constructor with upwards constrain
    Param<int> t5(1, 5, 10);
    assert(t5.val() == 5);
    assert(t5.min() == 5);
    assert(t5.max() == 10);
    assert(t5.last() == 5);
    assert(t5.saturated() == true);
    // test constructor with downwards constrain
    Param<int> t6(10, 1, 5);
    assert(t6.val() == 5);
    assert(t6.min() == 1);
    assert(t6.max() == 5);
    assert(t6.last() == 5);
    assert(t6.saturated() == true);

    // external limits constructor
    printf("Testing external limits constructor...\n");
    Param<int> t7(5, 0, 20);
    Param<int> t8(10, 0, 20);
    Param<int> t9(8, t7.shptr(), t8.shptr());
    assert(t9.val() == 8);
    assert(t9.min() == 5);
    assert(t9.max() == 10);
    assert(t9.last() == 8);
    assert(t9.saturated() == false);
    assert(t9.shptr().use_count() == 2);
    assert(t9.min_shptr().use_count() == 3); // 1 for each Param + 1 external
    assert(t9.max_shptr().use_count() == 3); // 1 for each Param + 1 external
    assert(t7.ptr() == t9.min_ptr());
    assert(t8.ptr() == t9.max_ptr());
    // test constructor with upwards constrain
    Param<int> t10(1, t7.shptr(), t8.shptr());
    assert(t10.val() == 5);
    assert(t10.min() == 5);
    assert(t10.max() == 10);
    assert(t10.last() == 5);
    assert(t10.saturated() == true);
    // test constructor with downwards constrain
    Param<int> t11(11, t7.shptr(), t8.shptr());
    assert(t11.val() == 10);
    assert(t11.min() == 5);
    assert(t11.max() == 10);
    assert(t11.last() == 10);
    assert(t11.saturated() == true);
    // test change limits
    Param<int> t12(8, t7.shptr(), t8.shptr());
    t7.set(4);
    t8.set(11);
    assert(t12.val() == 8);
    assert(t12.min() == 4);
    assert(t12.max() == 11);
    assert(t12.last() == 8);
    assert(t12.saturated() == false);
    // test change limits with upwards constrain
    t7.set(9);
    assert(t12.val() == 8);
    assert(t12.min() == 9);
    assert(t12.max() == 11);
    assert(t12.last() == 8);
    assert(t12.saturated() == false); // NOTE: should we constrain on get()...?
    t12.set(8);
    assert(t12.val() == 9);
    assert(t12.min() == 9);
    assert(t12.max() == 11);
    assert(t12.last() == 8);
    assert(t12.saturated() == true); // NOTE: should we constrain on get()...?
    // test change limits with downwards constrain
    t12.set(10);
    t7.set(1);
    t8.set(7);
    assert(t12.val() == 10);
    assert(t12.min() == 1);
    assert(t12.max() == 7);
    assert(t12.last() == 9);
    assert(t12.saturated() == false); // NOTE: should we constrain on get()...?
    t12.set(9);
    assert(t12.val() == 7);
    assert(t12.min() == 1);
    assert(t12.max() == 7);
    assert(t12.last() == 10);
    assert(t12.saturated() == true); // NOTE: should we constrain on get()...?
    // test external limits destructor
    Param<int> t13(10);
    {
        Param<int> p1(5);
        Param<int> p2(15);
        t13.set_limits(p1.shptr(), p2.shptr());
    }
    assert(t13.min() == 5);
    assert(t13.max() == 15);
    assert(t13.min_shptr().use_count() == 2);
    assert(t13.max_shptr().use_count() == 2);
}