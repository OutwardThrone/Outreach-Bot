package PIDStats;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.util.Date;

public class GatherStatistics {
	private PrintWriter writer;
	private Date time;
	private int count = 0;
	public GatherStatistics() throws IOException {
		count++;
		time = new Date();
		File f = new File("//home//PIDData//TurnPIDStats" + time.toString() +".txt");
		f.createNewFile();
		System.out.println("f exists" + f.exists() + f.setWritable(true));
		writer = new PrintWriter(f);		
		if(writer== null) System.out.println("Writer is null");
		System.out.println("Writer Exsists: " + writer != null);
		
		writer.println("Vernier Format 2");
		writer.println("Untiled.clmb 5/5/2019 9:37:43 .");
		writer.println("Data Set");
		writer.println("Time	Input	Output	Error");
		writer.println("x	y	z	w");

	}
	
	public void writeNewData(double seconds, double input, double output, double error) {
		writer.println("" + seconds + "\t" + input + "\t" + output + "\t" + error);
	}
	

}
//172.22.11.2