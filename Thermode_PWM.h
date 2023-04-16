template <class H>
bool check_range(H *out, H value, H min, H max)
{
	if (value < min)
	{
		return false;
	}
	if (value > max)
	{
		return false;
	}
	*out = value;
	return true;
}

template <class H>
bool check_range_abs(H *out, H value, H min, H max)
{
	if (abs(value) < min)
	{
		return false;
	}
	if (abs(value) > max)
	{
		return false;
	}
	*out = value;
	return true;
}
