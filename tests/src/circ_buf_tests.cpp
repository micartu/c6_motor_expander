#include "CppUTest/TestHarness.h"
#include "circ_buf.h"
#include "circ_dbuf.h"
#include <assert.h>

TEST_GROUP(CircularDoubleBuffer) {
  struct circ_buf_t obj;
  double values[5];

  void setup() {
    circ_double_buf_init(&obj, values, sizeof(values) / sizeof(*values));
  }

  void teardown() {}
};

TEST(CircularDoubleBuffer, PushValueAndGetItBack) {
  // given
  circ_double_buf_push(&obj, 1);
  circ_double_buf_push(&obj, 2);
  circ_double_buf_push(&obj, 3);

  // when
  double ret = circ_double_buf_value(&obj, 0);
  ret += circ_double_buf_value(&obj, 1);
  ret += circ_double_buf_value(&obj, 2);

  // then
  DOUBLES_EQUAL(6, ret, 0.1);
}

TEST(CircularDoubleBuffer, PushValueAndGetItBackRewriteFirst) {
  // given
  circ_double_buf_push(&obj, 1);
  circ_double_buf_push(&obj, 2);
  circ_double_buf_push(&obj, 3);
  circ_double_buf_push(&obj, 4);

  // when
  double ret = circ_double_buf_value(&obj, 0);
  ret += circ_double_buf_value(&obj, 1);
  ret += circ_double_buf_value(&obj, 2);

  // then
  DOUBLES_EQUAL(9, ret, 0.1);
}

TEST(CircularDoubleBuffer, PushValueAndGetItBackFromArray) {
  // given
  double vals[] = {1, 2, 3, 4, 5, 6, 7, 10, 13, 15};
  const size_t len = sizeof(vals) / sizeof(*vals);
  const size_t len2eval = 4;
  double exp = 0;
  assert(len2eval < len);
  assert(len2eval < obj.bufmaxind);
  for (int i = 0; i < len; ++i)
    circ_double_buf_push(&obj, vals[i]);
  for (int i = 0; i < len2eval; ++i)
    exp += vals[len - i - 1];

  // when
  double ret = 0;
  for (int i = 0; i < len2eval; ++i)
    ret += circ_double_buf_value(&obj, i);

  // then
  DOUBLES_EQUAL(exp, ret, 0.01);
}
