extern void test(double *a, double *b) {
	*a = 2 * (*a);
	*b = *a + *b;
}