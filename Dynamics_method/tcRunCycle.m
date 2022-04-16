function [progress, currentvel,newaccel]=tcRunCycle(progress,reqvel,currentvel,cycle_time  ,target  ,maxaccel,feed_override,maxvel)
   discr = 0.5 * cycle_time * currentvel - (target - progress);%倒算当前剩余多少路程没走
   if discr > 0.0
      %永远不会发生：意味着我们已经超过了目标
      newvel =0;
   else
      discr = 0.25 * (cycle_time)^2 - 2.0 / maxaccel * discr;%计算当前路径
      newvel = -0.5 * maxaccel * cycle_time + maxaccel * (discr)^0.5;%计算目前的新速度
   end
   
   if newvel < 0.0
      %也不应该发生-如果我们已经完成了这个tc，它就在上面被捕捉到了
      newvel = 0;
      progress = target;%表示已经完成
   else
     %约束速度
      if newvel > reqvel * feed_override
         newvel = reqvel * feed_override;
      end
      if newvel > maxvel
         newvel = maxvel;%end
      end
      %获得最终加速度
      newaccel = (newvel - currentvel) / cycle_time;
  
     %约束加速度并得到结果速度
      if newaccel > 0.0 && newaccel > maxaccel
         newaccel = maxaccel;
         newvel = currentvel + newaccel * cycle_time;
      end
      if newaccel < 0.0 && newaccel < -maxaccel
         newaccel = -maxaccel;
         newvel = currentvel + newaccel * cycle_time;
       end
      %更新此tc中的位置
      progress = progress+(newvel + currentvel) * 0.5 * cycle_time;
   end
   currentvel = newvel;
end