// Reporting for the tests. A test calls check() for each thing it checks, and returns check_result()
// from main().

#pragma once

#include <stdio.h>

static int check_failures;

static void check(const char* what, int ok)
{
    printf("%s  %s\n", ok ? "pass" : "FAIL", what);
    if (!ok)
        check_failures++;
}

static int check_result(void)
{
    printf("%s\n", check_failures ? "FAILED" : "ok");
    return check_failures ? 1 : 0;
}
