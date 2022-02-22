%!octave

function [moveString] = GetMove(move)

	run GlobalVar;

	switch move
		case PATH_MOVE_DECACCPEAK 
			moveString = "Dec/Acc";
		case PATH_MOVE_DECACCSATURATED 
			moveString = "Dec/Acc Saturated";
		case PATH_MOVE_ACCDECPEAK
			moveString = "Acc/Dec";
		case PATH_MOVE_ACCDECSATURATED
			moveString = "Acc/Dec Saturated";
		case PATH_MOVE_DECDEC
			moveString = "Dec/Dec";
		case PATH_MOVE_ACCACC
			moveString = "Acc/Acc";
		otherwise
			moveString = "Unknown";
	end

end % Function
