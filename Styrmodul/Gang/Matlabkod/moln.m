for(x=-200:10:200)
    for(y=-50:10:250)
        for(z=-100:10:250)
            [a,b,c]=test_vinklar(x,y,z);
            if(isreal(a) & isreal(b) & isreal(c))
                plot3(a,b,c)

                hold on
            end
        end
    end
end
xlabel('x')
                ylabel('y')
                zlabel('z')
                grid on
              
              