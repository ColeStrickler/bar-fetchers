package barf
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.subsystem.{CacheBlockBytes}
import org.chipsalliance.cde.config









class SingleOutRouter(nInputs: Int, nOutputs: Int, bitwidth: Int) extends Module 
{

    val NULL_ROUTE : Int = {
        val bits = log2Ceil(nOutputs + 1)
        (math.pow(2, bits)-1).toInt
    }

    val routerRegBitsNeeded = log2Ceil(NULL_ROUTE)
    val io = IO(new Bundle{
        val inputs = Input(Vec(nInputs, UInt(bitwidth.W)))
        val outputs = Output(Vec(nOutputs,UInt(bitwidth.W)))
        val routing = Input(Vec(nInputs, UInt(routerRegBitsNeeded.W))) // we need to map multiple inputs to each output
    })



    val buffer = WireInit(VecInit(Seq.fill(nInputs)(0.U(bitwidth.W))))
    for (i <- 0 until nInputs)
    {
        buffer(i) := io.inputs(i)
    }       

    for (i <- 0 until nOutputs) {
        io.outputs(i) := 0.U
    }
    // Route each output from the input specified in routing
    for (i <- 0 until nInputs) {
        
        val sel_output = io.routing(i)
        when (sel_output =/= NULL_ROUTE.U) // this is default value
        {               
            io.outputs(sel_output) := buffer(i)//io.inputs(i)
        }
          
    }
}







/*
class Snoop(implicit val p: Parameters) extends Bundle {
  val blockBytes = p(CacheBlockBytes)

  val write = Bool()
  val address = UInt()
  def block = address >> log2Up(blockBytes)
  def block_address = block << log2Up(blockBytes)
}

class Prefetch(implicit val p: Parameters) extends Bundle {
  val blockBytes = p(CacheBlockBytes)

  val write = Bool()
  val address = UInt()
  def block = address >> log2Up(blockBytes)
  def block_address = block << log2Up(blockBytes)
}

class PrefetcherIO(implicit p: Parameters) extends Bundle {
  val snoop = Input(Valid(new Snoop))
  val request = Decoupled(new Prefetch)
  val hit = Output(Bool())
}
*/



class SrcAddrEntry(addrBits: Int, idBits: Int) extends Bundle {
  val valid  = Bool()
  val source = UInt(idBits.W)
  val addr   = UInt(addrBits.W)
}

case class SelProjPrefetcherParams(
    val maxInFlight : Int = 4
) extends CanInstantiatePrefetcher {
  def desc() = "Sel-Proj Data Dependent Prefetcher"
  def instantiate()(implicit p: Parameters) = Module(new SelProjPrefetcher(this)(p))
}


object State extends ChiselEnum {
  val idle, dispatchStart, dispatchRows, dispatchEnd = Value
}


case class ProgrammableSelectionParams()(implicit p: Parameters)
{
  
}

case class ProgrammableSelectionIO(params: ProgrammableSelectionParams)(implicit p: Parameters) extends Bundle
{
  val row_columns = Input(Vec(16, UInt(64.W)))
  val valid_columns = Input(Vec(16, Bool()))
  val doPrefetch = Output(Bool())


  val config_RegOps = Input(Vec(16, Ops()))
  val config_ConstVal = Input(Vec(16, UInt(32.W)))
  val config_Routing = Input(Vec(32, UInt(log2Ceil(16 + 1).W))) // log2Ceil(nOutputs + 1)
}


object Ops extends ChiselEnum {
  val LT, GT, EQ = Value
}


class ProgrammableSelection (params : ProgrammableSelectionParams)(implicit p: Parameters) extends Module 
{

  val io = IO(new ProgrammableSelectionIO(params))

  val router = Module(new SingleOutRouter(32, 16, 64))

  val ConstRegs = Seq.fill(16)(Reg(UInt(32.W)))
  val OpRegs = Seq.fill(16)(Reg(Ops()))
  val results = Seq.fill(16)(Wire(Bool()))
  val comparison_val = Vec(16, Wire(UInt(64.W)))



  OpRegs.zipWithIndex.foreach { case (opReg, i) =>
    opReg := io.config_RegOps(i)
  }

  ConstRegs.zipWithIndex.foreach { case (constReg, i) =>
    constReg := io.config_ConstVal(i)  
  }

  router.io.routing := io.config_Routing


  for (i <- 0 until 16)
  {
    router.io.inputs(i) := io.row_columns(i)
  }
  for (i <- 16 until 32)
  {
    router.io.inputs(i) := ConstRegs(i).pad(64)
  }  


  comparison_val := router.io.outputs

  for (i <- 0 until 16) 
  {
    val op = OpRegs(i)
    switch (op)
    {
      is (Ops.LT)
      {
        results(i) := (io.row_columns(i) < comparison_val(i)) || !io.valid_columns(i)
      }

      is (Ops.GT)
      {
        results(i) := (io.row_columns(i) < comparison_val(i)) || !io.valid_columns(i)
      }
    }

  } 

  io.doPrefetch := results.reduce(_||_)

}


class PrefetchGenerator()(implicit p: Parameters) extends Module
{
    def isPower2(i : Int) : Boolean = {
      i > 0 && ((i & (i-1)) == 0)
    }

    def DivideBySmallConstantMuxTable(input: UInt, divisor: UInt, maxDivisor: Int) : UInt = {
        require(maxDivisor >= 1)
        MuxLookup(divisor,0.U(input.getWidth.W)) (
          (1 to maxDivisor).map { k =>
              val q = 
                if (isPower2(k)) (input >> log2Ceil(k)) else input / k.U


              k.U -> q
          })
    }

  val io = IO(new Bundle {
    val valid = Input(Bool())
    val numerator = Input(UInt(4.W))
    val denominator = Input(UInt(4.W))

    val sel_req_addr = Input(UInt(47.W))
    val selection_region = Input(UInt(47.W))
    val projection_region = Input(UInt(47.W))

    val prefetch_addr = Valid(Output(UInt(47.W)))
  })

  // can easily make these registers
  val sel_req_addr = WireInit(0.U(47.W))
  val selection_region = WireInit(0.U(47.W))
  val projection_region = WireInit(0.U(47.W))
  val sel_offset = WireInit(0.U(28.W))
  val valid_out = WireInit(false.B)

  valid_out := io.valid
  sel_offset := io.sel_req_addr - io.selection_region


  val afterDiv = DivideBySmallConstantMuxTable(sel_offset, io.denominator, 8)

  val proj_addr = (afterDiv * io.numerator) + projection_region

  
  io.prefetch_addr.bits := proj_addr
  io.prefetch_addr.valid := valid_out
}


class SelProjPrefetcher(params: SelProjPrefetcherParams)(implicit p: Parameters) extends AbstractPrefetcher(TLMonitorChannel.D)(p) {


  /*
    We can take the ratio of selection to projection columns to
    go from position in stream1 to stream to
  */
  val stream1Address = RegInit(0.U(47.W))
  val stream2Address = RegInit(0.U(47.W))
  val addrRatioNumerator = RegInit(0.U(4.W))
  val addrRatioDenominator = RegInit(0.U(4.W))
  val selColCount = RegInit(0.U(4.W))
  val rowBytes = RegInit(0.U(6.W))


  val prog_selection = Module(new ProgrammableSelection(ProgrammableSelectionParams()))
  val prefetchGenerator = Module(new PrefetchGenerator())
  
  val configIn = io.config.get


  // Forward config from MMIO
  prog_selection.io.config_ConstVal := configIn.conf_ConstVal
  prog_selection.io.config_RegOps := configIn.conf_Operations
  prog_selection.io.config_Routing := configIn.conf_Routing
  addrRatioDenominator := configIn.conf_addrRatioDenominator
  addrRatioNumerator := configIn.conf_addrRatioNumerator
  prefetchGenerator.io.selection_region := stream1Address
  prefetchGenerator.io.projection_region := stream2Address
  prefetchGenerator.io.numerator := addrRatioNumerator
  prefetchGenerator.io.denominator := addrRatioDenominator



      /*
        Since the TL-D channel does not have an address
    */
    val table = RegInit(VecInit(Seq.fill(params.maxInFlight)(0.U.asTypeOf( // tracks outbound requests
        new SrcAddrEntry(47, 10)
    ))))
    val freeOH = (~table.map(_.valid).asUInt)
    val hasFree = freeOH.orR
    val freeIdx = PriorityEncoder(freeOH)

    
    val fetchReqValid = io.snoop.valid 
    val fetchReplyValid = io.snoop.bits.snoopDChan.get.valid


    def isPower2(i : Int) : Boolean = {
      i > 0 && ((i & (i-1)) == 0)
    }

    def DivideBySmallConstantMuxTable(input: UInt, divisor: UInt, maxDivisor: Int) : UInt = {
        require(maxDivisor >= 1)
        MuxLookup(divisor,0.U(input.getWidth.W)) (
          (1 to maxDivisor).map { k =>
              val q = 
                if (isPower2(k)) (input >> log2Ceil(k)) else input / k.U


              k.U -> q
          })
    }


    def GetTableEntryIdx(src: UInt) : (Bool, UInt) = {
      val matches = table.map(e => e.valid && e.source === src)
      val hit = matches.reduce(_ || _)
      val idx = PriorityEncoder(matches)
      (hit, idx)
    }

    def GetTableEntrySrcMatch(src: UInt) : (Bool, SrcAddrEntry) = {
      val (hit, idx) = GetTableEntryIdx(src)
      val entry = Wire(new SrcAddrEntry(47, 10))
      entry := 0.U.asTypeOf(entry)
      entry := table(idx)

      (hit, entry)
    }

    def ClearEntrySrc(src: UInt) : Unit = {
      val (hit, idx) = GetTableEntryIdx(src)
      when (hit)
      {
        table(idx).valid := false.B
      }
    }


    def ExtractData(dataIn: Vec[UInt], byteStart: UInt, byteEnd: UInt, maxBytes: Int): UInt = { 
        assert(byteStart < byteEnd)
        assert(byteEnd <= dataIn.length.U)
       // require(dataIn.length == maxBytes)
        val shifted = Wire(Vec(maxBytes, UInt(8.W)))
        val nBytes = byteEnd - byteStart
        for (i <- 0 until maxBytes)
        {
          shifted(i) := Mux(i.U < nBytes, dataIn(i.U + byteStart), 0.U)
        }

        shifted.asUInt
    }






    /*
      Since the D channel has no address field
      we just track a source to address mapping here
    */
    when (fetchReqValid)
    {
      assert(hasFree) // try to always instantiate at MLP

      val newSrcEntry = Wire(new SrcAddrEntry(47, 10))
      newSrcEntry.addr := io.snoop.bits.address
      newSrcEntry.source := io.snoop.bits.source.get
      newSrcEntry.valid := true.B
      table(freeIdx) := newSrcEntry
    }




    /* 
        We can assemble an entire cache line first,
        and then operate on it.


        Lets separate into receive part on A channel and pre-fetch part on D channel
    */


  val dataReg = RegInit(UInt(512.W))
  val dataReg2 = RegInit(UInt(512.W))
  val dataAsLine = dataReg2.asTypeOf(Vec(64, UInt(8.W)))

  val dataWidth = io.snoop.bits.snoopDChan.get.bits.data.getWidth
  val shiftNewData = io.snoop.bits.snoopDChan.get.bits.data//+ d_count // count is to test
  val dataRegFull = RegInit(false.B)
  val addrReg = RegInit(0.U(47.W))
  val srcInDataReg = RegInit(0.U(10.W))
  dataRegFull := io.snoop.bits.snoopDChan.get.bits.done
  srcInDataReg := Mux(io.snoop.bits.snoopDChan.get.bits.done, io.snoop.bits.snoopDChan.get.bits.source, srcInDataReg)
      // we have to splice the data after shift because zeroes are put in the top



  /*
    When a full cache line is assembled, we pass dataReg to dataReg2 so we can start writing to
    dataReg for the next request immediately
  */
  dataReg := Mux(fetchReplyValid, Cat(shiftNewData, (dataReg >> dataWidth)((dataReg.getWidth - 1)-dataWidth, 0)), dataReg)
  dataReg2 := Mux(dataRegFull, dataReg, dataReg2)
  addrReg := Mux(dataRegFull, table(srcInDataReg).addr, addrReg)

  
  /*
    Free entries in table on return path


    We will pass out the address, and the table entry is no longer needed 
  */

  when (dataRegFull)
  {
    table(srcInDataReg) := 0.U.asTypeOf(new SrcAddrEntry(47, 10))
  }


  /*
    Here we are simply finding out where the rows are aligned inside the current cache line
    such that we can operate on them
  */
  val advanceLut = VecInit((0 to 64).map { rb =>
    if (rb == 0) 0.U(6.W) else (64 % rb).U(6.W)
  })

  val periodLut = VecInit((0 to 64).map { rb =>
    if (rb == 0) 1.U(6.W)
    else (rb / BigInt(rb).gcd(BigInt(64)).toInt).U(6.W)
  })
  def modBySmallConstMux(num: UInt, mod: UInt, maxMod: Int): UInt = { // the synthesizer knows the divisor at compile time
    MuxLookup(mod, 0.U)(
      (1 to maxMod).map { k =>
        k.U -> (num % k.U)
      }
    )
}


val addr = addrReg
val lineIndex = addr >> 6 // 2
val advance   = advanceLut(rowBytes) // 64 % 12 = 4
val period    = periodLut(rowBytes)  // 4
val idxInPattern = modBySmallConstMux(lineIndex, period, 63) // 2 % 4 = 2
val phaseRaw = idxInPattern * advance // 2 * 4 = 8

/*
Suppose: phase = 8
Then from byte zero:
  Line byte:     0  1  2  3  4  5  6  7 ...
  Row byte:      8  9 10 11  0  1  2  3 ...
*/
val phase = modBySmallConstMux(phaseRaw, rowBytes, 64)  // 8 % 12 = 8
val start_end_phase = rowBytes - phase

/*
  Once we know where the rows are,
  we can extract them one at a time
*/

val state = RegInit(State.idle)
val rowExtractValid = WireInit(false.B)
val start = RegInit(0.U(6.W))
val rowExtract = Wire(Vec(64, UInt(8.W)))
switch (state)
{
  is (State.idle)
  {
    rowExtract := 0.U.asTypeOf(Vec(64, UInt(8.W)))
    rowExtractValid := false.B
    state := Mux(dataRegFull, State.dispatchStart, State.idle)
  }

  is (State.dispatchStart)
  {
    rowExtract :=  ExtractData(dataAsLine, 0.U, phase, 64).asTypeOf(Vec(64, UInt(8.W)))
    rowExtractValid := true.B
    start := start + phase
    state := State.dispatchRows
  }

  is (State.dispatchRows)
  {
    rowExtract :=  ExtractData(dataAsLine, start, start+rowBytes, 64).asTypeOf(Vec(64, UInt(8.W)))
    start := start+rowBytes
    rowExtractValid := true.B
    assert(start + rowBytes <= 64.U)
    when (64.U-(start+rowBytes) < rowBytes)
    {
      state := State.dispatchRows
    } .otherwise
    {
      state := State.dispatchEnd
    }
  }

  is (State.dispatchEnd)
  {
    rowExtract := ExtractData(dataAsLine, start, rowBytes, 64).asTypeOf(Vec(64, UInt(8.W)))
    rowExtractValid:= true.B
    state := State.idle
  }
}

/*
    Now given an extracted row, we extract the columns
*/



val columnSize = 4.U
val maxExtracts = 16
val extracted_row_columns = (0 until maxExtracts).map { i =>
  val start = (start_end_phase + columnSize*i.U)
  val end = start+columnSize
  val valid = (start+end <= rowBytes)
  val data = Mux(valid, ExtractData(rowExtract,start,end, 8),
    0.U)

  (valid, data)
}



extracted_row_columns.zipWithIndex.foreach { case ((valid, data), i) =>
  prog_selection.io.row_columns(i) := data
  prog_selection.io.valid_columns(i) := valid
}




def MakePrefetch(addr: UInt) : Prefetch = {
  val prefetch = Wire(new Prefetch())
  prefetch.address := addr
  prefetch.write := false.B
  prefetch
}


val doPrefetch = prog_selection.io.doPrefetch


prefetchGenerator.io.valid := doPrefetch
val prefetchAddr = prefetchGenerator.io.prefetch_addr


val PrefetchRequest = MakePrefetch(prefetchAddr.bits)

io.request.bits := PrefetchRequest
io.request.valid := prefetchAddr.valid
}