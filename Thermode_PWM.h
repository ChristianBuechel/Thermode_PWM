template<class H>
H check_range_def(H value, H min, H max, H def) {
    if (value < min) return def;  
	if (value > max) return def;
	return value;
	
}