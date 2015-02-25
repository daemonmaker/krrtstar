function sigma = solve_gamma(n, lambda, epsilon)
    bottom = 0;
    top = 15;
 
    terminate = false;

    %sigma = (top - bottom)/2 + bottom
    sigma = 1.1774
    while terminate == false
        res = gammainc((sigma^2)/2.0, n/2.0) - (1-lambda)

        if abs(res) < epsilon
            return
        end
        
        if res > 0
            top = sigma
            sigma = (sigma - bottom)/2 + bottom
        else
            bottom = sigma
            sigma = (top - sigma)/2 + bottom
        end
    end