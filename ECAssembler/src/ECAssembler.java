import java.util.HashMap;
import java.util.LinkedList;

import java.io.*;
class parseException extends Throwable{
	public parseException(String msg){
		super(msg);
	}
}
class ParserHelper{
	static int parseRegister(String reg)throws parseException{
		  try{
			if (reg.matches("^(R|r)\\d\\d?$")) //General Register r*
				return Integer.parseInt(reg.substring(1));
			throw new parseException(reg+" is not a legal register.");
		  }catch(Exception e) {
			throw new parseException(reg+" is not a legal register.");
		  }
		}
	static int parseFlagRegister(String reg) throws parseException{
		if      (reg.equalsIgnoreCase("zero"))
			return 0;
		else if (reg.equalsIgnoreCase("one"))
			return 1;
		else if (reg.equalsIgnoreCase("zf"))
			return 2;
		else if (reg.equalsIgnoreCase("if"))
			return 3;
		else if (reg.equalsIgnoreCase("nzf"))
			return 4;
		else if (reg.equalsIgnoreCase("nif"))
			return 5;
		else if (reg.equalsIgnoreCase("rnd"))
			return 6;
		else
			throw new parseException(reg+" is not a legal general flag register.");
	}
	
}

abstract class AbstractInstructionParser {
	abstract protected String[] getAcceptedTokens();
	
	boolean matchInstruction(String[] tokens) {
		String[] acceptedTokens = getAcceptedTokens();
		for(int i=0;i<acceptedTokens.length;i++)
			if (tokens[0].equalsIgnoreCase(acceptedTokens[i]))
				return true;
		return false;
	}
	
	abstract int compile(String[] tokens, int numTokens, int lineNo, LabelManager labels) throws parseException;
	
}

class LdoParser extends AbstractInstructionParser{
	protected String[] getAcceptedTokens(){
		String[] s = {"ldo"};
		return s;
	}
	int compile(String[] tokens, int numTokens, int lineNo, LabelManager labels) throws parseException{
		 if (numTokens != 3) throw new parseException("Incorrect number of tokens");
		 return 0*(1<<8) + ParserHelper.parseRegister(tokens[1])*(1<<4)+
		 				   ParserHelper.parseRegister(tokens[2])*(1<<0);
	}
}
class ArithParser extends AbstractInstructionParser{
	protected String[] getAcceptedTokens(){
		String[] s = {"mul", "inv", "sub", "tstz", "mov", "memrd", "adrld", "output", "tstdp", "sbo"};
		return s;
	}
	int compile(String[] tokens, int numTokens, int lineNo, LabelManager labels) throws parseException{
		 if (numTokens != 2) throw new parseException("Incorrect number of tokens");
		 int arithID;
		 if      (tokens[0].equalsIgnoreCase("sub")) arithID = 0;
		 else if (tokens[0].equalsIgnoreCase("mul")) arithID = 1;
		 else if (tokens[0].equalsIgnoreCase("inv")) arithID = 2;
		 else if (tokens[0].equalsIgnoreCase("mov")) arithID = 3;
		 else if (tokens[0].equalsIgnoreCase("tstz")) arithID = 4;
		 else if (tokens[0].equalsIgnoreCase("memrd")) arithID = 5;
		 else if (tokens[0].equalsIgnoreCase("output")) arithID = 6;
		 else if (tokens[0].equalsIgnoreCase("adrld")) arithID = 7;
		 else if (tokens[0].equalsIgnoreCase("tstdp")) arithID = 8;
		 else if (tokens[0].equalsIgnoreCase("sbo")) arithID = 9;
		 else throw new parseException("Unknown arithmatic operator: "+tokens[0]);
		 return 1*(1<<8) + arithID*(1<<4)+
		 				   ParserHelper.parseRegister(tokens[1])*(1<<0);
	}
}
class JmpParser extends AbstractInstructionParser{
	protected String[] getAcceptedTokens(){
		String[] s = {"jz", "jnz"};
		return s;
	}
	int compile(String[] tokens, int numTokens, int lineNo, LabelManager labels) throws parseException{
		 if (numTokens != 2) throw new parseException("Incorrect number of tokens");
		 return (tokens[0].equalsIgnoreCase("jz")?2:3)*(1<<8) +
		 		((labels.getLabelLineNo(tokens[1]) - lineNo)&0xFF);
		 	
	}
}
class FlagParser extends AbstractInstructionParser{
	protected String[] getAcceptedTokens(){
		String[] s = {"setflg"};
		return s;
	}
	int compile(String[] tokens, int numTokens, int lineNo, LabelManager labels) throws parseException{
		 if (numTokens != 3) throw new parseException("Incorrect number of tokens");
		 int dstflg = ParserHelper.parseFlagRegister(tokens[1]);
		 if ( (dstflg != ParserHelper.parseFlagRegister("ZF"))&&(dstflg != ParserHelper.parseFlagRegister("IF")))
			 throw new parseException("Invalid target flag register: "+tokens[1]);
		 return 4*(1<<8) + ParserHelper.parseFlagRegister(tokens[2])*(1<<4)+
		 				   ParserHelper.parseFlagRegister(tokens[1])*(1<<0);
	}
}
class AdrGEParser extends AbstractInstructionParser{
	protected String[] getAcceptedTokens(){
		String[] s = {"adrge"};
		return s;
	}
	int compile(String[] tokens, int numTokens, int lineNo, LabelManager labels) throws parseException{
		 if (numTokens != 2) throw new parseException("Incorrect number of tokens");
		 return 5*(1<<8) + (Integer.parseInt(tokens[1])&0xFF); 
	}
}
class AdrIncParser extends AbstractInstructionParser{
	protected String[] getAcceptedTokens(){
		String[] s = {"adrinc"};
		return s;
	}
	int compile(String[] tokens, int numTokens, int lineNo, LabelManager labels) throws parseException{
		 if (numTokens != 2) throw new parseException("Incorrect number of tokens");
		 return 6*(1<<8) +  (Integer.parseInt(tokens[1])&0xFF); 
	}
}
class LabelManager{
	HashMap<String,Integer> labels;
	LabelManager(){
		labels = new HashMap<String,Integer>();
	}
	
	void addLabel(String label, int line) throws parseException{
		if (labels.get(label) != null)
			throw new parseException("Duplicated label defined: "+label);
		labels.put(label, line);
		return;
	}
	int getLabelLineNo(String label)throws parseException{
		Integer l;
		if ((l=labels.get(label)) == null)
			throw new parseException("Refer to undefined label: "+label);
		return l.intValue();
	}
}

class Assembler{
	BufferedReader src;
	FileWriter dst;
	final int MAX_FILESIZE= 32*1024;
	AbstractInstructionParser[] ips;
	int numIP = 0;
	public Assembler(String srcFile, String destFile) throws parseException{
		try {
			ips = new AbstractInstructionParser[20];
			dst = new FileWriter(destFile, false);
			src = new BufferedReader(new FileReader(srcFile));
		}catch (Exception e){
			throw new parseException(""+e);
		}
	}
	
	public void addInstructionParser(AbstractInstructionParser ip){
		ips[numIP++] = ip;
	}
	public void compile() throws parseException{
		LabelManager labels = new LabelManager();
		StreamTokenizer tk;
		String[] tokens = new String[10]; 
		int lineNo = 0;
		try {
			src.mark(MAX_FILESIZE);
			String line;
			while ((line = src.readLine()) != null){ //Phase 1, Parse labels
				if (line.length() == 0) continue;
				lineNo++;
				tk = new StreamTokenizer(new StringReader(line));
				tk.slashSlashComments(true);tk.whitespaceChars(',', ',');
				tk.wordChars('0', '9');tk.wordChars('@','@');tk.wordChars('_','_');
				tk.nextToken();
				if (tk.sval.charAt(0) =='@')
					labels.addLabel(tk.sval.substring(1), lineNo);
			}
			//Phase 2, Main Parsing
			src.reset();
			lineNo = 0;
			while ((line = src.readLine()) != null){ 
				if (line.length() == 0) continue;
				lineNo++;
				System.out.println(line);
				tk = new StreamTokenizer(new StringReader(line));
				tk.slashSlashComments(true);tk.whitespaceChars(',', ',');
				tk.wordChars('0', '9');tk.wordChars('@','@');tk.wordChars('_','_');
				int numTokens = 0;
				while (tk.nextToken() != StreamTokenizer.TT_EOF){//Build a token list
					if ((numTokens ==0)&&(tk.sval.charAt(0)=='@')) continue;
					tokens[numTokens++] = (tk.ttype == StreamTokenizer.TT_WORD) ? tk.sval :
							(tk.ttype == StreamTokenizer.TT_NUMBER) ? String.format("%.0f",tk.nval) :
							"Unknown Token??";
				}
				//Test if any parser can handle this line
				AbstractInstructionParser curParser = null;
				for(int i=0;i<numIP;i++)
					if (ips[i].matchInstruction(tokens))
						if (curParser == null)
							curParser = ips[i];
						else
							throw new parseException("Ambiguous instruction.");
				//parse current line and write back
				int opcode = curParser.compile(tokens, numTokens, lineNo, labels);
				String binary = Integer.toBinaryString(opcode); while (binary.length() != 11) binary = "0"+binary;
				String opc = String.format("\t8'd%d:\tcode <= 11'b%s; //%s\n", 
						lineNo-1,binary, line);
				dst.write(opc);
			}
			dst.flush();
		}catch (Exception e){
			throw new parseException("Internal Exception: Line "+lineNo+": "+e);
		}catch (parseException e){
			throw new parseException("Line "+lineNo+": "+e);
		}
	}
}
public class ECAssembler {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		float x = 1.0f;
		while (x+1 != x) { x= x*2.0f;}
		System.out.println(x);
		System.out.println(x-1);
		System.out.println("ECAssembler for Cyclone II EC Processor.");
		if (args.length != 2) {
			System.out.println("usage: ECAssembler [src.txt] [dest.txt]");
			System.exit(0);
		}
		try {
			Assembler asm = new Assembler(args[0], args[1]);
			asm.addInstructionParser(new LdoParser());
			asm.addInstructionParser(new ArithParser());
			asm.addInstructionParser(new JmpParser());
			asm.addInstructionParser(new FlagParser());
			asm.addInstructionParser(new AdrGEParser());
			asm.addInstructionParser(new AdrIncParser());
			asm.compile();
			System.out.println("Compilation successful.");
		}catch (parseException e){
			System.out.println(e);
		}
	}

	//public static <T extends Long>   void myMethod(LinkedList<T> x){return;}
	//public static <T extends String> void myMethod(LinkedList<T> x){return;}
	
}
